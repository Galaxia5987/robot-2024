package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    default void updateInputs(SwerveDriveInputs inputs) {}

    default Rotation2d getYaw() {return new Rotation2d();}

    default Rotation2d getRawYaw() {
        return new Rotation2d();
    }

    default Rotation2d getPitch() {
        return new Rotation2d();
    }

    default void resetGyro(Rotation2d angle) {}
}
