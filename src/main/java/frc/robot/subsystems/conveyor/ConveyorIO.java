package frc.robot.subsystems.conveyor;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {

    ConveyorInputsAutoLogged inputs = new ConveyorInputsAutoLogged();

    void setPower(double power);

    @AutoLog
    class ConveyorInputs {
        public double setpointPower = 0;
        public MutableMeasure<Voltage> appliedVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> appliedCurrent = MutableMeasure.zero(Units.Amps);
    }
}