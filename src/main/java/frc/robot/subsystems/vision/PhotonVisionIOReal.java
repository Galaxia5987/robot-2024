package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Optional;
import java.util.OptionalDouble;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIOReal implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final Transform3d robotToCamera;
    private final boolean calculateScoreParams;
    private boolean isNoteDetector;
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
    private OptionalDouble yawToNote = OptionalDouble.empty();
    private final String name;

    public PhotonVisionIOReal(
            PhotonCamera camera,
            String name,
            Transform3d robotToCamera,
            AprilTagFieldLayout field,
            boolean calculateScoreParams,
            boolean isNoteDetector) {
        this.camera = camera;
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.calculateScoreParams = calculateScoreParams;
        this.isNoteDetector = isNoteDetector;
        camera.setPipelineIndex(0);
        estimator =
                new PhotonPoseEstimator(
                        field,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        camera,
                        robotToCamera);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public OptionalDouble getYawToNote() {
        return yawToNote;
    }

    @Override
    public Optional<ScoreParameters> getScoreParameters() {
        return scoreParameters;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = camera.isConnected();

        var latestResult = camera.getLatestResult();

        if (isNoteDetector) {
            inputs.yawNote = (camera.getLatestResult().getBestTarget().getYaw());
            if (latestResult.hasTargets()) {
                yawToNote = OptionalDouble.of(inputs.yawNote);
            } else {
                yawToNote = OptionalDouble.empty();
            }
        }

        var estimatedPose = estimator.update(latestResult);
        if (estimatedPose.isPresent()) {
            inputs.poseFieldOriented = estimatedPose.get().estimatedPose;
            if (calculateScoreParams) {
                var toSpeaker =
                        inputs.poseFieldOriented
                                .getTranslation()
                                .toTranslation2d()
                                .minus(VisionConstants.getSpeakerPose());
                toSpeaker = new Translation2d(toSpeaker.getX(), toSpeaker.getY());
                inputs.distanceToSpeaker = toSpeaker.getNorm();

                var centerTag =
                        latestResult.getTargets().stream()
                                .filter(
                                        (target) ->
                                                target.getFiducialId()
                                                        == VisionConstants.getSpeakerTag1())
                                .toList();
                if (!centerTag.isEmpty()) {
                    var yaw = centerTag.get(0).getYaw();
                    inputs.yawToSpeaker = Rotation2d.fromDegrees(yaw);
                }
                scoreParameters =
                        Optional.of(
                                new ScoreParameters(inputs.distanceToSpeaker, Optional.empty()));
            } else {
                scoreParameters = Optional.empty();
            }

            result = new VisionResult(estimatedPose.get(), true);
            double distanceTraveled =
                    result.getEstimatedRobotPose()
                            .estimatedPose
                            .minus(lastResult.getEstimatedRobotPose().estimatedPose)
                            .getTranslation()
                            .getNorm();
            if ((DriverStation.isEnabled() && distanceTraveled > 0.2)
                    || (result.getEstimatedRobotPose().estimatedPose.getZ() > 0.1)
                    || (VisionConstants.outOfBounds(result.getEstimatedRobotPose().estimatedPose))
                    || result.getEstimatedRobotPose().targetsUsed.stream()
                            .anyMatch(
                                    (target) ->
                                            target.getBestCameraToTarget()
                                                            .getTranslation()
                                                            .getNorm()
                                                    > 5.0)) {
                result.setUseForEstimation(false);
            }
        } else {
            result.setUseForEstimation(false);
        }

        lastResult = result;
    }

    @Override
    public VisionResult getLatestResult() {
        return result;
    }

    @Override
    public Transform3d getCameraToRobot() {
        return robotToCamera;
    }

    @Override
    public String getName() {
        return name;
    }
}
