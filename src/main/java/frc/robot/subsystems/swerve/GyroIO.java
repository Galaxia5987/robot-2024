package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    void updateInputs(SwerveDriveInputs inputs);

    Rotation2d getYaw();

    default Rotation2d getRawYaw() {
        return new Rotation2d();
    }

    default Rotation2d getPitch() {
        return new Rotation2d();
    }

    void resetGyro(Rotation2d angle);
}
