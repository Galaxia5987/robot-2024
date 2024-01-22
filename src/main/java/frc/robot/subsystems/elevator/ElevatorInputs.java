package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
public class ElevatorInputs implements LoggableInputs {
    public double power;
    public MutableMeasure<Distance> height;
    public MutableMeasure<Distance> heightSetpoint;
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration;
    public ElevatorIO.ControlMode controlMode;

    @Override
    public void toLog(LogTable table) {

    }

    @Override
    public void fromLog(LogTable table) {

    }
}
