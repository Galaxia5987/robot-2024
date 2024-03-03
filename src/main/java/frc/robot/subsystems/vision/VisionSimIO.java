package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.lib.Utils;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
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
    private EstimatedRobotPose result;
    private AprilTagFieldLayout tagFieldLayout;

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
    }
}
