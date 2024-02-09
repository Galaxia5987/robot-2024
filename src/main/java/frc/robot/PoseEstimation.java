package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.swerve.SwerveDrive;
import frc.robot.vision.Vision;

public class PoseEstimation {
    private static PoseEstimation INSTANCE = null;
    private final SwerveDrivePoseEstimator estimator;
    private final Vision vision = Vision.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public PoseEstimation() {
        estimator =
                new SwerveDrivePoseEstimator(
                        swerveDrive.getKinematics(),
                        swerveDrive.getYaw(),
                        swerveDrive.getModulePositions(),
                        swerveDrive.getBotPose());
    }

    public static PoseEstimation getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PoseEstimation();
        }
        return INSTANCE;
    }

    public void addVisionMeasurement() {
        var results = vision.getResults();
        for (org.photonvision.EstimatedRobotPose result : results) {
            estimator.addVisionMeasurement(
                    result.estimatedPose.toPose2d(),
                    result.timestampSeconds,
                    VecBuilder.fill(0.0, 0.0, 0.0)); // TODO: calibrate
        }
    }

    public void updatePose() {
        estimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions());
    }

    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPosition();
    }
}
