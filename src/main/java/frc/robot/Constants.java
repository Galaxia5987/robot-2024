package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
