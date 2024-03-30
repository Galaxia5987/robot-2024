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
    private final boolean isNoteDetector;
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
            var tags = latestResult.getTargets();
            inputs.distanceToTargets = new double[tags.size()];
            for (int i = 0; i < tags.size(); i++) {
                inputs.distanceToTargets[i] = tags.get(i).getBestCameraToTarget().getTranslation().getNorm();
            }
            inputs.timestamp = estimatedPose.get().timestampSeconds;
            inputs.hasNewPose = true;
        } else {
            inputs.hasNewPose = false;
        }

        if (calculateScoreParams && latestResult.hasTargets()) {
            inputs.toSpeaker = inputs.poseFieldOriented
                    .getTranslation()
                    .toTranslation2d()
                    .minus(VisionConstants.getSpeakerPose());
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
