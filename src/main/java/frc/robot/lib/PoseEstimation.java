package frc.robot.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
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
            Supplier<Stream<Double>> distances =
                    () ->
                            result.targetsUsed.stream()
                                    .map(
                                            (target) ->
                                                    target.getBestCameraToTarget()
                                                            .getTranslation()
                                                            .getNorm());
            if (distances.get().anyMatch((d) -> d > 2.5)) {
                continue;
            }
            var ambiguities = distances.get().map((d) -> d * d);
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
