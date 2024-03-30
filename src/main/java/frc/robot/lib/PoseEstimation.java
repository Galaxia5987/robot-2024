package frc.robot.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionResult;
import java.util.Arrays;
import java.util.Optional;
import java.util.stream.DoubleStream;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseEstimation {
    private static PoseEstimation INSTANCE = null;

    private final SwerveDrive swerveDrive;
    private final Vision vision;

    private Translation2d speakerPose;

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
        if (results == null) {
            return;
        }
        for (VisionResult result : results) {
            if (result == null || !result.useForEstimation()) {
                continue;
            }
            DoubleStream distances = Arrays.stream(result.distanceToTargets());
            var ambiguities = distances.map((d) -> d * d);
            double stddev = multiplier * Utils.averageAmbiguity(ambiguities);
            swerveDrive
                    .getEstimator()
                    .addVisionMeasurement(
                            result.estimatedRobotPose().toPose2d(),
                            result.timestamp(),
                            VecBuilder.fill(
                                    stddev,
                                    stddev,
                                    stddev
                                            * SwerveConstants.MAX_OMEGA_VELOCITY
                                            / SwerveConstants.MAX_X_Y_VELOCITY));
        }
    }

    @AutoLogOutput(key = "Robot/EstimatedRobotPose")
    public Pose2d getEstimatedPose() {
        return swerveDrive.getEstimator().getEstimatedPosition();
    }

    @AutoLogOutput(key = "Shooter/DistanceToSpeaker")
    public double getDistanceToSpeaker() {
        return getPoseRelativeToSpeaker().getNorm();
    }

    @AutoLogOutput(key = "Robot/ToSpeaker")
    public Translation2d getPoseRelativeToSpeaker() {
        Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
        if (allianceColor.isPresent() && (allianceColor.get() == DriverStation.Alliance.Red)) {
            speakerPose = Constants.SPEAKER_POSE_RED;
        } else {
            speakerPose = Constants.SPEAKER_POSE_BLUE;
        }
        return speakerPose.minus(getEstimatedPose().getTranslation());
    }
}
