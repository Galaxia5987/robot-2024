package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.*;
import org.littletonrobotics.junction.AutoLogOutput;

public class Constants {

    public static final int CONFIG_TIMEOUT = 100; // [ms]

    public static final double LOOP_TIME = 0.02;
    public static final double AUTO_VISION_MEASUREMENT_MULTIPLIER = 0.5;
    public static final double AUTO_START_VISION_MEASUREMENT_MULTIPLIER = 1_000_000_000;
    public static final double TELEOP_VISION_MEASUREMENT_MULTIPLIER = 0.05;
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
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = Units.MetersPerSecond.of(2.0);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
            Units.MetersPerSecondPerSecond.of(1);
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
        0.791_302_619_782_565_4,
        0.025_138_350_628_458_764,
        0.281_357_657_033_941_44,
        0.574_898_689_372_467_2
    };

    public static Mode CURRENT_MODE = Mode.REAL;

    @AutoLogOutput
    static double VISION_MEASUREMENT_MULTIPLIER = AUTO_START_VISION_MEASUREMENT_MULTIPLIER;

    private static double yawToNote = 0;

    public static void initSwerve() {
        ModuleIO[] moduleIOs = new ModuleIO[4];
        GyroIO gyroIO =
                switch (CURRENT_MODE) {
                    case REAL -> {
                        moduleIOs[0] =
                                new ModuleIOTalonFX(
                                        Ports.SwerveDrive.DRIVE_IDS[0],
                                        Ports.SwerveDrive.ANGLE_IDS[0],
                                        Ports.SwerveDrive.ENCODER_IDS[0],
                                        SwerveConstants.DRIVE_MOTOR_CONFIGS,
                                        SwerveConstants.FRONT_LEFT_ANGLE_MOTOR_CONFIGS,
                                        new SwerveModuleInputsAutoLogged());
                        for (int i = 1; i < moduleIOs.length; i++) {
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
                    case SIM -> {
                        for (int i = 0; i < moduleIOs.length; i++) {
                            moduleIOs[i] = new ModuleIOSim(new SwerveModuleInputsAutoLogged());
                        }
                        yield new GyroIOSim();
                    }
                    default -> {
                        for (int i = 0; i < moduleIOs.length; i++) {
                            moduleIOs[i] = new ModuleIO() {};
                        }
                        yield new GyroIO() {};
                    }
                };
        SwerveDrive.initialize(gyroIO, SWERVE_OFFSETS, moduleIOs);
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        AutoBuilder.configureHolonomic(
                () -> swerveDrive.getEstimator().getEstimatedPosition(),
                swerveDrive::resetPose,
                swerveDrive::getCurrentSpeeds,
                (speeds) -> {
                    //                     Fixes diversion from note during autonomous
                    //                    if (RobotContainer.getInstance().useNoteDetection) {
                    //                        if (Vision.getInstance().getYawToNote().isPresent()) {
                    //                            yawToNote =
                    // Vision.getInstance().getYawToNote().get().getSin();
                    //                        }
                    //                        speeds.vyMetersPerSecond =
                    //
                    // SwerveConstants.VY_NOTE_DETECTION_CONTROLLER.calculate(yawToNote);
                    //                    }

                    swerveDrive.drive(speeds, false);
                },
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.5, 0, 0.15),
                        new PIDConstants(3, 0, 0.4),
                        SwerveConstants.MAX_X_Y_VELOCITY,
                        Constants.ROBOT_LENGTH.in(Units.Meters) / Math.sqrt(2),
                        new ReplanningConfig()),
                Constants::isRed,
                swerveDrive);
    }

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public static final Translation2d SPEAKER_POSE_BLUE = new Translation2d(0, 5.547_944_2);
    public static final Translation2d SPEAKER_POSE_RED =
            GeometryUtil.flipFieldPosition(SPEAKER_POSE_BLUE);

    public enum State {
        SHOOT,
        AMP,
        CLIMB
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
