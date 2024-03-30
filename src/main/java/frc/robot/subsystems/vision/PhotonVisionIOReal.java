package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
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
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = camera.isConnected();

        var latestResult = camera.getLatestResult();

        if (isNoteDetector && latestResult.hasTargets()) {
            inputs.yawNote = Rotation2d.fromDegrees(latestResult.getBestTarget().getYaw());
            inputs.hasNote = true;
        } else {
            inputs.hasNote = false;
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

        if (calculateScoreParams && latestResult.hasTargets()) {
            var toSpeaker =
                    inputs.poseFieldOriented
                            .getTranslation()
                            .toTranslation2d()
                            .minus(VisionConstants.getSpeakerPose());
            inputs.toSpeaker = toSpeaker;
            var centerTag = latestResult.getTargets();
            centerTag.removeIf(
                    (target) -> target.getFiducialId() != VisionConstants.getSpeakerTag1());
            if (!centerTag.isEmpty()) {
                inputs.yawToSpeaker = Rotation2d.fromDegrees(-centerTag.get(0).getYaw());
                inputs.seesSpeaker = true;
            } else {
                inputs.seesSpeaker = false;
            }
            inputs.hasScoreParams = true;
        } else {
            inputs.hasScoreParams = false;
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
