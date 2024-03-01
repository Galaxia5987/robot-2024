package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIOReal implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final PhotonPoseEstimator simpleEstimator;
    private final Transform3d robotToCamera;
    private final AprilTagFieldLayout field;
    private final boolean calculateScoreParams;
    private VisionResult result =
            new VisionResult(
                    new EstimatedRobotPose(
                            new Pose3d(),
                            0,
                            new ArrayList<>(),
                            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                    false);
    private VisionResult lastResult =
            new VisionResult(
                    new EstimatedRobotPose(
                            new Pose3d(),
                            0,
                            new ArrayList<>(),
                            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                    false);
    private Optional<ScoreParameters> scoreParameters = Optional.empty();
    private final LinearFilter distanceFilter = LinearFilter.movingAverage(20);

    public PhotonVisionIOReal(
            PhotonCamera camera,
            Transform3d robotToCamera,
            AprilTagFieldLayout field,
            boolean calculateScoreParams) {
        this.camera = camera;
        this.robotToCamera = robotToCamera;
        this.field = field;
        this.calculateScoreParams = calculateScoreParams;
        camera.setPipelineIndex(0);
        estimator =
                new PhotonPoseEstimator(
                        field,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        camera,
                        robotToCamera);

        simpleEstimator =
                new PhotonPoseEstimator(
                        field,
                        PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
                        camera,
                        robotToCamera);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public Optional<ScoreParameters> getScoreParameters() {
        return scoreParameters;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = camera.isConnected();

        var latestResult = camera.getLatestResult();

        if (calculateScoreParams) {
            var robotPose = simpleEstimator.update(latestResult);
            if (robotPose.isPresent()) {
                var toSpeaker =
                        robotPose
                                .get()
                                .estimatedPose
                                .getTranslation()
                                .toTranslation2d()
                                .minus(VisionConstants.getSpeakerPose());
                inputs.distanceToSpeaker = distanceFilter.calculate(toSpeaker.getNorm());
                inputs.yawToSpeaker =
                        new Rotation2d(
                                Math.IEEEremainder(
                                        Math.atan2(toSpeaker.getY(), toSpeaker.getX()),
                                        2 * Math.PI));
                System.out.println(toSpeaker.getY());
                scoreParameters =
                        Optional.of(
                                new ScoreParameters(inputs.distanceToSpeaker, inputs.yawToSpeaker));
            } else {
                scoreParameters = Optional.empty();
            }
        }

        var estimatedPose = estimator.update(latestResult);
        if (estimatedPose.isPresent()) {
            inputs.poseFieldOriented = estimatedPose.get().estimatedPose;

            result = new VisionResult(estimatedPose.get(), true);
            double distanceTraveled =
                    result.getEstimatedRobotPose()
                            .estimatedPose
                            .minus(lastResult.getEstimatedRobotPose().estimatedPose)
                            .getTranslation()
                            .getNorm();
            if ((DriverStation.isEnabled() && distanceTraveled > 0.3)
                    || (result.getEstimatedRobotPose().estimatedPose.getZ() > 0.2)
                    || (VisionConstants.outOfBounds(
                            result.getEstimatedRobotPose().estimatedPose))) {
                result.setUseForEstimation(false);
            }
        } else {
            result.setUseForEstimation(false);
        }

        lastResult = result;
    }

    @Override
    public EstimatedRobotPose getLatestResult() {
        return result.getEstimatedRobotPose();
    }

    @Override
    public Transform3d getCameraToRobot() {
        return robotToCamera;
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
