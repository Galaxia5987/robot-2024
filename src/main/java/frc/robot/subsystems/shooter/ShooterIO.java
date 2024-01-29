package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    void setTopVelocity(MutableMeasure<Velocity<Angle>> velocity);

    void setBottomVelocity(MutableMeasure<Velocity<Angle>> velocity);

    void stop();

    /** Update the inputs of the Shooter */
    void updateInputs();

    @AutoLog
    class ShooterInputs {
        public MutableMeasure<Velocity<Angle>> topVelocity =
                MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> bottomVelocity =
                MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> topVelocitySetpoint =
                MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> bottomVelocitySetpoint =
                MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Voltage> topVoltage = MutableMeasure.zero(Volts);
        public MutableMeasure<Voltage> bottomVoltage = MutableMeasure.zero(Volts);
    }
}
