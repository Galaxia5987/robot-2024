package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.swerve.SwerveDrive;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class SimVisionSystem {
    private static SimVisionSystem INSTANCE;
    private static final VisionSystemSim visionSim = new VisionSystemSim("main");
    private AprilTagFieldLayout tagFieldLayout;

    private SimVisionSystem(AprilTagFieldLayout fieldLayout) {
        try {
            tagFieldLayout = fieldLayout;
        } catch (Exception e) {
            return;
        }
        visionSim.addAprilTags(tagFieldLayout);
    }

    public static SimVisionSystem getInstance(PhotonCameraSim cameraSim, Transform3d robotToCam) {
        visionSim.addCamera(cameraSim, robotToCam);
        return INSTANCE;
    }

    public static SimVisionSystem getInstance() {
        return INSTANCE;
    }

    public static void initialize(AprilTagFieldLayout fieldLayout) {
        INSTANCE = new SimVisionSystem(fieldLayout);
    }

    public void update() {
        visionSim.update(SwerveDrive.getInstance().getBotPose());
    }
}
