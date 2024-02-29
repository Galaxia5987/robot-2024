package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
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
        inputs.isConnected = camera.isConnected();

        var latestResult = camera.getLatestResult();

        if (latestResult != null) {
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
