package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public interface ModuleIO {
    void updateInputs();

    SwerveModuleInputsAutoLogged getInputs();

    void updatePID();

    default boolean hasPIDChanged(LoggedTunableNumber[] PIDValues) {
        boolean hasChanged = false;
        for (LoggedTunableNumber value : PIDValues) {
            if (value.hasChanged(hashCode())) hasChanged = true;
        }
        return hasChanged;
    }

    Rotation2d getAngle();

    void setAngle(Rotation2d angle);

    double getVelocity();

    void setVelocity(double velocity);

    SwerveModuleState getModuleState();

    SwerveModulePosition getModulePosition();

    void stop();

    Command checkModule();

    default void updateOffset(Rotation2d offset) {}
}
