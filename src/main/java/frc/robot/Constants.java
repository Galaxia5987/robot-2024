package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import frc.robot.swerve.*;
import frc.robot.vision.PhotonVisionIOReal;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionModule;
import frc.robot.vision.VisionSimIO;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

public class Constants {

    public static final int CONFIG_TIMEOUT = 100; // [ms]

    public static final Mode CURRENT_MODE = Mode.SIM;

    public static final Transform3d BACK_LEFT_CAMERA_POSE =
            new Transform3d(
                    -0.293,
                    0.293,
                    0.2,
                    new Rotation3d(0, Math.toRadians(31.92), Math.toRadians(180)));
    public static final Transform3d BACK_RIGHT_CAMERA_POSE =
            new Transform3d(
                    -0.293,
                    -0.293,
                    0.2,
                    new Rotation3d(0, Math.toRadians(31.92), Math.toRadians(-100)));
    public static final Transform3d FRONT_LEFT_CAMERA_POSE =
            new Transform3d(0, 0.293, 0.55, new Rotation3d(0, Math.toRadians(10.0), 0));
    public static final Transform3d FRONT_RIGHT_CAMERA_POSE =
            new Transform3d(0, -0.293, 0.55, new Rotation3d(0, Math.toRadians(25.0), 0));

    public static final Measure<Distance> ROBOT_LENGTH = Units.Meters.of(0.584);
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = Units.MetersPerSecond.of(4.5);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
            Units.MetersPerSecondPerSecond.of(3);
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
                                            SwerveConstants.ANGLE_MOTOR_CONFIGS);
                        }
                        yield new GyroIOReal();
                    }
                    default -> {
                        for (int i = 0; i < moduleIOs.length; i++) {
                            moduleIOs[i] = new ModuleIOSim();
                        }
                        yield new GyroIOSim();
                    }
                };
        SwerveDrive.initialize(gyroIO, moduleIOs);
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        AutoBuilder.configureHolonomic(
                swerveDrive::getBotPose,
                swerveDrive::resetPose,
                swerveDrive::getCurrentSpeeds,
                (speeds) -> swerveDrive.drive(speeds, false),
                new HolonomicPathFollowerConfig(
                        SwerveConstants.MAX_X_Y_VELOCITY,
                        Constants.ROBOT_LENGTH.in(Units.Meters) / 2,
                        new ReplanningConfig()),
                () -> false,
                swerveDrive);
    }

    public static void initVision() {
        VisionModule orangePi1;
        VisionModule orangePi2 =
                switch (CURRENT_MODE) {
                    case REAL -> {
                        orangePi1 =
                                new VisionModule(
                                        new PhotonVisionIOReal(
                                                new PhotonCamera("Front left camera"),
                                                FRONT_LEFT_CAMERA_POSE,
                                                AprilTagFields.k2024Crescendo
                                                        .loadAprilTagLayoutField()),
                                        new PhotonVisionIOReal(
                                                new PhotonCamera("Front right camera"),
                                                FRONT_RIGHT_CAMERA_POSE,
                                                AprilTagFields.k2024Crescendo
                                                        .loadAprilTagLayoutField()));
                        yield new VisionModule(
                                new PhotonVisionIOReal(
                                        new PhotonCamera("Back left camera"),
                                        BACK_LEFT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()),
                                new PhotonVisionIOReal(
                                        new PhotonCamera("Back right camera"),
                                        BACK_RIGHT_CAMERA_POSE,
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()));
                    }
                    default -> {
                        orangePi1 =
                                new VisionModule(
                                        new VisionSimIO(
                                                new PhotonCamera("Front left camera"),
                                                FRONT_LEFT_CAMERA_POSE,
                                                AprilTagFields.k2024Crescendo
                                                        .loadAprilTagLayoutField(),
                                                SimCameraProperties.PI4_LIFECAM_640_480()),
                                        new VisionSimIO(
                                                new PhotonCamera("Front right camera"),
                                                FRONT_RIGHT_CAMERA_POSE,
                                                AprilTagFields.k2024Crescendo
                                                        .loadAprilTagLayoutField(),
                                                SimCameraProperties.PI4_LIFECAM_640_480()));
                        yield new VisionModule(
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
                    }
                };
        Vision.initialize(orangePi1, orangePi2);
    }
}
