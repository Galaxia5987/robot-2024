package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    record ScoreParameters(Translation2d toSpeaker, Optional<Rotation2d> yaw) {}

    default void setPipeLine(int pipeLineIndex) {}

    default void updateInputs(VisionInputs inputs) {}

    default VisionResult getLatestResult() {
        return null;
    }

    default Transform3d getCameraToRobot() {
        return new Transform3d();
    }

    default String getName() {
        return "Camera";
    }

    @AutoLog
    class VisionInputs {
        public Pose3d poseFieldOriented = new Pose3d();
        public boolean isConnected = false;
        public boolean seesSpeaker = false;
        public boolean hasScoreParams = false;
        public Translation2d toSpeaker = new Translation2d();
        public Rotation2d yawToSpeaker = new Rotation2d();
        public boolean hasNote = false;
        public Rotation2d yawNote = new Rotation2d();
    }
}
