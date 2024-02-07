package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.swerve.SwerveDrive;
import frc.robot.vision.Vision;

public class PoseEstimation {
    private static PoseEstimation INSTANCE = null;
    private SwerveDrivePoseEstimator estimator;
    private Vision vision = Vision.getInstance();
    private SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public PoseEstimation(){
        estimator = new SwerveDrivePoseEstimator(
                swerveDrive.getKinematics(),
                swerveDrive.getYaw(),
                swerveDrive.getModulePositions(),
                swerveDrive.getBotPose());
    }

    public static PoseEstimation getInstance(){
        if (INSTANCE==null){
            INSTANCE = new PoseEstimation();
        }
        return INSTANCE;
    }

    public void addVisionMeasurement(){
        var results = vision.getResults();
        for (int i = 0; i < results.length; i++) {
            estimator.addVisionMeasurement(
                    results[i].estimatedPose.toPose2d(),
                    results[i].timestampSeconds,
                    VecBuilder.fill(0.0, 0.0, 0.0)); //TODO: calibrate
        }
    }

    public void updatePose(){
        estimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions());
    }

    public Pose2d getEstimatedPose(){
        return estimator.getEstimatedPosition();
    }
}
