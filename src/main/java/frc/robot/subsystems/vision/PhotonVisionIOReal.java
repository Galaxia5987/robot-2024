package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
    private final Transform3d robotToCamera;
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
    private final AprilTagFieldLayout field;
    private Optional<ScoreParameters> scoreParameters = Optional.empty();
    private final boolean calculateScoreParams;

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
        try {
            estimator =
                    new PhotonPoseEstimator(
                            field,
                            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            camera,
                            robotToCamera);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
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
            var tags = latestResult.getTargets();
            boolean seesTag1 =
                    tags.stream()
                            .anyMatch(
                                    (tag) ->
                                            VisionConstants.getSpeakerTag1()
                                                    == tag.getFiducialId());
            boolean seesTag2 =
                    tags.stream()
                            .anyMatch(
                                    (tag) ->
                                            VisionConstants.getSpeakerTag2()
                                                    == tag.getFiducialId());
            inputs.seesSpeaker = seesTag1 && seesTag2;
            if (inputs.seesSpeaker) {
                var translationToTag1 = tags.get(0).getBestCameraToTarget().plus(robotToCamera);
                var translationToTag2 = tags.get(1).getBestCameraToTarget().plus(robotToCamera);
                var robot1 =
                        field.getTagPose(VisionConstants.getSpeakerTag1())
                                .get()
                                .plus(translationToTag1)
                                .plus(robotToCamera);
                var robot2 =
                        field.getTagPose(VisionConstants.getSpeakerTag2())
                                .get()
                                .plus(translationToTag2)
                                .plus(robotToCamera);
                var robotPose = robot1.plus(robot2.minus(robot1).div(2.0));
                var toSpeaker =
                        robotPose
                                .getTranslation()
                                .toTranslation2d()
                                .minus(VisionConstants.getSpeakerPose());
                inputs.distanceToSpeaker = toSpeaker.getNorm();
                inputs.yawToSpeaker = new Rotation2d(toSpeaker.getX(), toSpeaker.getY());

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
