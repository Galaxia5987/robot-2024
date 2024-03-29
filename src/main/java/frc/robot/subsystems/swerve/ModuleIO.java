package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public interface ModuleIO {
    default void updateInputs() {}

    default SwerveModuleInputsAutoLogged getInputs() {
        return new SwerveModuleInputsAutoLogged();
    }

    default void updatePID() {}

    default boolean hasPIDChanged(LoggedTunableNumber[] PIDValues) {
        boolean hasChanged = false;
        for (LoggedTunableNumber value : PIDValues) {
            if (value.hasChanged()) hasChanged = true;
        }
        return hasChanged;
    }

    default Rotation2d getAngle() {
        return new Rotation2d();
    }

    default void setAngle(Rotation2d angle) {}

    default double getVelocity() {
        return 0;
    }

    default void setVelocity(double velocity) {}

    default SwerveModuleState getModuleState() {
        return new SwerveModuleState();
    }

    default SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition();
    }

    default void stop() {}

    default Command checkModule() {
        return Commands.none();
    }

    default void updateOffset(Rotation2d offset) {}

    default void setVoltage(double volts) {}
}
