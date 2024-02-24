package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.*;
import frc.robot.scoreStates.ScoreState;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.PhotonVisionIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionModule;
import frc.robot.subsystems.vision.VisionSimIO;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

public class Constants {

    public static final int CONFIG_TIMEOUT = 100; // [ms]

    public static Mode CURRENT_MODE = Mode.REAL;

    public static double VISION_MEASUREMENT_MULTIPLIER = 1;

    public static final Transform3d BACK_LEFT_CAMERA_POSE =
            new Transform3d(
                    -0.289_36,
                    0.341_15,
                    0.2,
                    new Rotation3d(0, -Math.toRadians(31.92), Math.toRadians(170)));
    public static final Transform3d BACK_RIGHT_CAMERA_POSE =
            new Transform3d(
                    -0.346_52,
                    -0.285_32,
                    0.2,
                    new Rotation3d(0, -Math.toRadians(31.92), -Math.toRadians(90)));
    public static final Transform3d FRONT_LEFT_CAMERA_POSE =
            new Transform3d(0.061, 0.2848, 0.55, new Rotation3d(0, -Math.toRadians(10.0), 0));
    public static final Transform3d FRONT_RIGHT_CAMERA_POSE =
            new Transform3d(0.061, -0.2848, 0.55, new Rotation3d(0, -Math.toRadians(25.0), 0));

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

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final double[] SWERVE_OFFSETS = {
        0.794_376_719_859_418,
        0.780_963_919_524_097_9,
        0.484_072_612_101_815_3,
        0.609_018_215_225_455_4
    };

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
                (pose)->{},
                swerveDrive::getCurrentSpeeds,
                (speeds) -> swerveDrive.drive(speeds, false),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5, 0, 0),
                        new PIDConstants(5, 0, 0),
                        SwerveConstants.MAX_X_Y_VELOCITY,
                        Constants.ROBOT_LENGTH.in(Units.Meters) / Math.sqrt(2),
                        new ReplanningConfig()),
                ScoreState::isRed,
                swerveDrive);
    }

    public static void initVision() {
        VisionModule rightOpi;
        VisionModule leftOpi;
        switch (CURRENT_MODE) {
            case REAL:
                rightOpi =
                        new VisionModule(
                                new PhotonVisionIOReal(
                                        new PhotonCamera("Front_right_camera"),
                                        FRONT_RIGHT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()),
                                new PhotonVisionIOReal(
                                        new PhotonCamera("Back_right_camera"),
                                        BACK_RIGHT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()));
                leftOpi =
                        new VisionModule(
                                new PhotonVisionIOReal(
                                        new PhotonCamera("Back_left_camera"),
                                        BACK_LEFT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()));
                break;
            default:
                rightOpi =
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera("Front left camera"),
                                        FRONT_LEFT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                                        SimCameraProperties.PI4_LIFECAM_640_480()),
                                new VisionSimIO(
                                        new PhotonCamera("Front right camera"),
                                        FRONT_RIGHT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                                        SimCameraProperties.PI4_LIFECAM_640_480()));
                leftOpi =
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera("Back left camera"),
                                        BACK_LEFT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                                        SimCameraProperties.PI4_LIFECAM_640_480()),
                                new VisionSimIO(
                                        new PhotonCamera("Back right camera"),
                                        BACK_RIGHT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                                        SimCameraProperties.PI4_LIFECAM_640_480()));
                break;
        }
        Vision.initialize(rightOpi, leftOpi);
    }
}
