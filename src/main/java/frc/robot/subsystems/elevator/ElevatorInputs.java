package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {
    public double power = 0;
    public MutableMeasure<Distance> carriageHeight = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Distance> gripperHeight = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Distance> heightSetpoint = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration =
            Units.MetersPerSecondPerSecond.of(0).mutableCopy();
    public ElevatorIO.ControlMode controlMode = ElevatorIO.ControlMode.POSITION;
    public boolean isBottom = true;
    public boolean isTop = false;
}
