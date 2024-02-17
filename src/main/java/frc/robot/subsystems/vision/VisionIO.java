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
        long latency = 0;
        boolean hasTargets = false;
        double yaw = 0;
        double pitch = 0;
        double area = 0;
        double targetSkew = 0;
        long targetID = 0;
        Pose3d cameraToTarget = new Pose3d();
        Pose3d poseFieldOriented = new Pose3d();
        double bestTargetAmbiguity = 0;
        public double ambiguity = 0;
    }
}
