package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

public class LimelightIO implements VisionIO {

    private final String name;
    private final boolean calculateScoreParams;
    private VisionResult result =
            new VisionResult(
                            new Pose3d(),
                            0,
                    new double[0],
                    false);

    public LimelightIO(String name, boolean calculateScoreParams) {
        this.name = name;
        this.calculateScoreParams = calculateScoreParams;
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        LimelightHelpers.setPipelineIndex(name, pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        if (calculateScoreParams && LimelightHelpers.getTV(name)) {
            inputs.poseFieldOriented = LimelightHelpers.getBotPose3d_wpiBlue(name);
            inputs.toSpeaker =
                    inputs.poseFieldOriented
                            .getTranslation()
                            .toTranslation2d()
                            .minus(VisionConstants.getSpeakerPose());
            inputs.yawToSpeaker = Rotation2d.fromDegrees(LimelightHelpers.getTX(name));
            inputs.seesSpeaker = true;
        } else {
            inputs.seesSpeaker = false;
        }
    }

    @Override
    public Transform3d getCameraToRobot() {
        return new Transform3d();
    }

    @Override
    public String getName() {
        return name;
    }
}
