package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class VisionConstants {

    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.23;

    public static boolean outOfBounds(Pose3d estimatedPose) {
        return estimatedPose.getX() < 0 || estimatedPose.getX() > FIELD_LENGTH
                || estimatedPose.getY() < 0 || estimatedPose.getY() > FIELD_WIDTH;
    }
}
