package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {

    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputs inputs);

    EstimatedRobotPose getLatestResult();

    Transform3d getCameraToRobot();

    String getName();

    @AutoLog
    class VisionInputs {
        Pose3d poseFieldOriented = new Pose3d();
    }
}
