package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    record ScoreParameters(double distanceToSpeaker, Optional<Rotation2d> yaw) {}

    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputs inputs);

    VisionResult getLatestResult();

    Transform3d getCameraToRobot();

    String getName();

    default Optional<ScoreParameters> getScoreParameters() {
        return Optional.empty();
    }

    @AutoLog
    class VisionInputs {
        public Pose3d poseFieldOriented = new Pose3d();
        public boolean isConnected = false;
        public boolean seesSpeaker = false;
        public double distanceToSpeaker;
        public Rotation2d yawToSpeaker;
    }
}
