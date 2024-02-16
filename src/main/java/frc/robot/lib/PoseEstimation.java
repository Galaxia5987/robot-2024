package frc.robot.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseEstimation {
    private static PoseEstimation INSTANCE = null;

    private final SwerveDrive swerveDrive;
    private final Vision vision;

    private PoseEstimation() {
        swerveDrive = SwerveDrive.getInstance();
        vision = Vision.getInstance();
    }

    public static PoseEstimation getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PoseEstimation();
        }
        return INSTANCE;
    }

    public void processVisionMeasurements(double multiplier) {
        var results = vision.getResults();
        for (org.photonvision.EstimatedRobotPose result : results) {
            if (result == null) {
                continue;
            }
            var ambiguities =
                    result.targetsUsed.stream()
                            .map(
                                    (target) ->
                                            Math.pow(
                                                    target.getBestCameraToTarget()
                                                            .getTranslation()
                                                            .getNorm(),
                                                    2));
            double stddev =
                    multiplier * Utils.averageAmbiguity(ambiguities.collect(Collectors.toList()));
            swerveDrive
                    .getEstimator()
                    .addVisionMeasurement(
                            result.estimatedPose.toPose2d(),
                            result.timestampSeconds,
                            VecBuilder.fill(
                                    stddev,
                                    stddev,
                                    stddev
                                            * (SwerveConstants.MAX_X_Y_VELOCITY
                                                    / SwerveConstants.MAX_OMEGA_VELOCITY)));
        }
    }

    @AutoLogOutput(key = "EstimatedRobotPose")
    public Pose2d getEstimatedPose() {
        return swerveDrive.getEstimator().getEstimatedPosition();
    }
}
