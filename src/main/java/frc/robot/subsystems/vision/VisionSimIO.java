package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.Utils;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSimIO implements VisionIO {
    private final PhotonCamera photonCamera;
    private final PhotonCameraSim cameraSim;
    private final Transform3d robotToCam;
    private SimVisionSystem simVisionSystem;
    private AprilTagFieldLayout tagFieldLayout;
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

    public VisionSimIO(
            PhotonCamera photonCamera,
            Transform3d robotToCam,
            AprilTagFieldLayout tagFieldLayout,
            SimCameraProperties properties) {
        this.robotToCam = robotToCam;
        this.photonCamera = photonCamera;
        this.tagFieldLayout = tagFieldLayout;
        cameraSim = new PhotonCameraSim(photonCamera, properties);
        cameraSim.setMaxSightRange(5);
        simVisionSystem = SimVisionSystem.getInstance(cameraSim, robotToCam);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {}

    @Override
    public VisionResult getLatestResult() {
        return new VisionResult(result, true);
    }

    @Override
    public Transform3d getCameraToRobot() {
        return robotToCam;
    }

    @Override
    public String getName() {
        return photonCamera.getName();
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        var pose = SwerveDrive.getInstance().getBotPose();
        var pose3d = Utils.pose2dToPose3d(pose);
        PhotonPipelineResult latestResult =
                cameraSim.process(
                        0,
                        pose3d.plus(robotToCam.div(-1)),
                        tagFieldLayout.getTags().stream()
                                .map(
                                        (a) ->
                                                new VisionTargetSim(
                                                        a.pose, TargetModel.kAprilTag36h11, a.ID))
                                .collect(Collectors.toList()));
        cameraSim.submitProcessedFrame(latestResult);







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
}
