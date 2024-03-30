package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    default void updateInputs(SwerveDriveInputs inputs) {}

    default void resetGyro(Rotation2d angle) {}
}
