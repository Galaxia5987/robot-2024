package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.*;
import frc.robot.scoreStates.ScoreState;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

public class Constants {

    public static final int CONFIG_TIMEOUT = 100; // [ms]

    public static final double LOOP_TIME = 0.02;
    public static final double AUTO_VISION_MEASUREMENT_MULTIPLIER = 0.5;
    public static final double AUTO_START_VISION_MEASUREMENT_MULTIPLIER = 1_000_000_000;
    public static final double TELEOP_VISION_MEASUREMENT_MULTIPLIER = 0.5;
    public static final Transform3d SPEAKER_RIGHT_CAMERA_POSE =
            new Transform3d(
                    -0.065_833,
                    -0.040_05,
                    0.608_178,
                    new Rotation3d(0, -Math.toRadians(25), Math.toRadians(180)));
    public static final Transform3d SPEAKER_LEFT_CAMERA_POSE =
            new Transform3d(
                    -0.065_833,
                    0.039_95,
                    0.608_178,
                    new Rotation3d(0, -Math.toRadians(25), Math.toRadians(180)));
    public static final Transform3d INTAKE_APRILTAG_CAMERA_POSE =
            new Transform3d(
                    -0.012_852, -0.0005, 0.635_282, new Rotation3d(0, -Math.toRadians(25), 0));
    public static final Transform3d DRIVER_CAMERA_POSE =
            new Transform3d(0.0, 0.0, 0.53, new Rotation3d(0, Math.toRadians(20), 0));
    public static final Measure<Voltage> NOMINAL_VOLTAGE = Units.Volts.of(12);
    public static final Measure<Distance> ROBOT_LENGTH = Units.Meters.of(0.584);
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = Units.MetersPerSecond.of(4);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
            Units.MetersPerSecondPerSecond.of(2.5);
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
            Units.RotationsPerSecond.of(
                    MAX_VELOCITY.in(Units.MetersPerSecond)
                            / (ROBOT_LENGTH.in(Units.Meters) / Math.sqrt(2)));
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION =
            Units.RotationsPerSecond.per(Units.Second)
                    .of(
                            MAX_ACCELERATION.in(Units.MetersPerSecondPerSecond)
                                    / (ROBOT_LENGTH.in(Units.Meters) / Math.sqrt(2)));
    public static final PathConstraints AUTO_CONSTRAINTS =
            new PathConstraints(
                    MAX_VELOCITY.in(Units.MetersPerSecond),
                    MAX_ACCELERATION.in(Units.MetersPerSecondPerSecond),
                    MAX_ANGULAR_VELOCITY.in(Units.RotationsPerSecond),
                    MAX_ANGULAR_ACCELERATION.in(Units.RotationsPerSecond.per(Units.Second)));
    public static final double[] SWERVE_OFFSETS = {
        0.794_517_619_862_940_4,
        0.782_804_769_570_119_2,
        0.523_762_213_094_055_4,
        0.577_792_614_444_815_3
    };

    public static Mode CURRENT_MODE = Mode.REAL;

    @AutoLogOutput
    static double VISION_MEASUREMENT_MULTIPLIER = AUTO_START_VISION_MEASUREMENT_MULTIPLIER;

    public static void initSwerve() {
        ModuleIO[] moduleIOs = new ModuleIO[4];
        GyroIO gyroIO =
                switch (CURRENT_MODE) {
                    case REAL -> {
                        for (int i = 0; i < moduleIOs.length; i++) {
                            moduleIOs[i] =
                                    new ModuleIOTalonFX(
                                            Ports.SwerveDrive.DRIVE_IDS[i],
                                            Ports.SwerveDrive.ANGLE_IDS[i],
                                            Ports.SwerveDrive.ENCODER_IDS[i],
                                            SwerveConstants.DRIVE_MOTOR_CONFIGS,
                                            SwerveConstants.ANGLE_MOTOR_CONFIGS,
                                            new SwerveModuleInputsAutoLogged());
                        }
                        yield new GyroIOReal();
                    }
                    default -> {
                        for (int i = 0; i < moduleIOs.length; i++) {
                            moduleIOs[i] = new ModuleIOSim(new SwerveModuleInputsAutoLogged());
                        }
                        yield new GyroIOSim();
                    }
                };
        SwerveDrive.initialize(gyroIO, SWERVE_OFFSETS, moduleIOs);
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        AutoBuilder.configureHolonomic(
                () -> swerveDrive.getEstimator().getEstimatedPosition(),
                swerveDrive::resetPose,
                swerveDrive::getCurrentSpeeds,
                (speeds) -> {
                    // Fixes diversion from note during autonomous
                    if (RobotContainer.getInstance().isIntaking
                            && Vision.getInstance().getYawToNote().isPresent()) {
                        speeds.vyMetersPerSecond =
                                SwerveConstants.vyOffsetControllerAutonomous.calculate(
                                        Math.sin(
                                                Math.toRadians(
                                                        Vision.getInstance()
                                                                .getYawToNote()
                                                                .getAsDouble())),
                                        0);
                    }

                    swerveDrive.drive(speeds, false);
                },
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.5, 0, 0.15),
                        new PIDConstants(3, 0, 0.4),
                        SwerveConstants.MAX_X_Y_VELOCITY,
                        Constants.ROBOT_LENGTH.in(Units.Meters) / Math.sqrt(2),
                        new ReplanningConfig()),
                ScoreState::isRed,
                swerveDrive);
    }

    public static void initVision() {
        VisionModule rightOpi;
        VisionModule leftOpi;
        VisionIO speakerLeftCamera;
        VisionIO speakerRightCamera;
        VisionIO intakeAprilTagCamera;
        VisionIO driverCamera;
        var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        switch (CURRENT_MODE) {
            case REAL:
                intakeAprilTagCamera =
                        new PhotonVisionIOReal(
                                new PhotonCamera("OV2311_0"),
                                "Intake_AprilTag_Camera",
                                INTAKE_APRILTAG_CAMERA_POSE,
                                field,
                                false,
                                false);
                speakerRightCamera =
                        new PhotonVisionIOReal(
                                new PhotonCamera("OV2311_1"),
                                "Speaker_Right_Camera",
                                SPEAKER_RIGHT_CAMERA_POSE,
                                field,
                                true,
                                false);
                speakerLeftCamera =
                        new PhotonVisionIOReal(
                                new PhotonCamera("OV2311_2"),
                                "Speaker_Left_Camera",
                                SPEAKER_LEFT_CAMERA_POSE,
                                field,
                                true,
                                false);
                //                driverCamera =
                //                        new PhotonVisionIOReal(
                //                                new PhotonCamera("Driver_Camera"),
                //                                "Driver_Camera",
                //                                DRIVER_CAMERA_POSE,
                //                                field,
                //                                false,
                //                                true);
                break;
            default:
                speakerLeftCamera =
                        new VisionSimIO(
                                new PhotonCamera("Speaker_Left_Camera"),
                                SPEAKER_LEFT_CAMERA_POSE,
                                field,
                                SimCameraProperties.LL2_1280_720());
                speakerRightCamera =
                        new VisionSimIO(
                                new PhotonCamera("Speaker_Right_Camera"),
                                SPEAKER_RIGHT_CAMERA_POSE,
                                field,
                                SimCameraProperties.LL2_1280_720());
                intakeAprilTagCamera =
                        new VisionSimIO(
                                new PhotonCamera("Intake_AprilTag_Camera"),
                                INTAKE_APRILTAG_CAMERA_POSE,
                                field,
                                SimCameraProperties.LL2_1280_720());
                driverCamera =
                        new VisionSimIO(
                                new PhotonCamera("Driver_Camera"),
                                DRIVER_CAMERA_POSE,
                                field,
                                SimCameraProperties.LL2_1280_720());
                break;
        }
        rightOpi =
                new VisionModule(
                        //                driverCamera,
                        intakeAprilTagCamera);
        leftOpi = new VisionModule(speakerLeftCamera, speakerRightCamera);
        Vision.initialize(rightOpi, leftOpi);
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
