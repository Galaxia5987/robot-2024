package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    /**
     * Sets the position of the hood.
     *
     * @param velocity The position of the subsystem.
     */
    void setVelocity(MutableMeasure<Velocity<Angle>> velocity);

    /** Update the inputs of the Shooter */
    void updateInputs();

    @AutoLog
    class ShooterInputs {
        public MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> velocitySetpoint =
                MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
    }
}
