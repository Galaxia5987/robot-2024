package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {
    public double power;
    public MutableMeasure<Distance> height;
    public MutableMeasure<Distance> heightSetpoint;
    public double acceleration;
    public ElevatorIO.ControlMode controlMode;
}
