package frc.robot.subsystems.conveyor;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {

    ConveyorInputsAutoLogged inputs = new ConveyorInputsAutoLogged();

    void setVelocity(MutableMeasure<Velocity<Angle>> velocity);

    void updateInputs();

    @AutoLog
    class ConveyorInputs {
        public MutableMeasure<Velocity<Angle>> velocitySetpoint =
                MutableMeasure.zero(Units.RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> currentVelocity =
                MutableMeasure.zero(Units.RotationsPerSecond);
        public MutableMeasure<Voltage> appliedVoltage = MutableMeasure.zero(Units.Volts);
    }
}
