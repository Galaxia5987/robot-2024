package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIOReal implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final Transform3d robotToCamera;
    private EstimatedRobotPose result;

    public PhotonVisionIOReal(
            PhotonCamera camera, Transform3d robotToCamera, AprilTagFieldLayout field) {
        this.camera = camera;
        this.robotToCamera = robotToCamera;
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
    public void updateInputs(VisionInputs inputs) {
        var latestResult = camera.getLatestResult();

        if (latestResult != null) {
            var estimatedPose = estimator.update(latestResult);
            if (estimatedPose.isPresent()) {
                inputs.poseFieldOriented = estimatedPose.get().estimatedPose;

                result = estimatedPose.get();
            } else {
                result = null;
            }
        } else {
            result = null;
        }
    }

    @Override
    public EstimatedRobotPose getLatestResult() {
        return result;
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
