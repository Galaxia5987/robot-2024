package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    record ScoreParameters(
            Translation2d toSpeaker, Optional<Rotation2d> yaw, Rotation2d alternateYaw) {}

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

    default Optional<ScoreParameters> getScoreParameters() {
        return Optional.empty();
    }

    default OptionalDouble getYawToNote() {
        return OptionalDouble.empty();
    }

    @AutoLog
    class VisionInputs {
        public Pose3d poseFieldOriented = new Pose3d();
        public boolean isConnected = false;
        public boolean seesSpeaker = false;
        public double distanceToSpeaker;
        public Rotation2d yawToSpeaker;
        public double yawNote = 0;
    }
}
