package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {
    public double power;
    public MutableMeasure<Distance> height;
    public MutableMeasure<Distance> heightSetpoint;
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration;
    public ElevatorIO.ControlMode controlMode;
}
