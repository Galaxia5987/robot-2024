package frc.robot.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseEstimation {
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveDrive swerveDrive;
    private final Vision vision;

    public PoseEstimation(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        vision = Vision.getInstance();
        estimator =
                new SwerveDrivePoseEstimator(
                        swerveDrive.getKinematics(),
                        swerveDrive.getYaw(),
                        swerveDrive.getModulePositions(),
                        swerveDrive.getBotPose());
    }

    public void addVisionMeasurement(
            DoubleSupplier std1, DoubleSupplier std2, DoubleSupplier std3) {
        var results = vision.getResults();
        for (org.photonvision.EstimatedRobotPose result : results) {
            estimator.addVisionMeasurement(
                    result.estimatedPose.toPose2d(),
                    result.timestampSeconds,
                    VecBuilder.fill(std1.getAsDouble(), std2.getAsDouble(), std3.getAsDouble()));
        }
    }

    public void updatePose() {
        swerveDrive.updateHighFreqPose(estimator);
    }

    public void resetPose(Pose2d pose) {
        estimator.resetPosition(
                swerveDrive.getRawYaw(),
                swerveDrive.getModulePositions(),
                pose);
    }

    @AutoLogOutput(key = "EstimatedRobotPose")
    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPosition();
    }
}
