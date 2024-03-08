package frc.robot.subsystems.elevator;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorInputs {
    public MutableMeasure<Distance> hooksHeight = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Distance> carriageHeight = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Distance> heightSetpoint = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration =
            Units.MetersPerSecondPerSecond.of(0).mutableCopy();
    public ElevatorIO.ControlMode controlMode = ElevatorIO.ControlMode.POSITION;
    public boolean isBottom = true;
    public MutableMeasure<Angle> stopperAngle = Units.Degrees.of(0).mutableCopy();
    public MutableMeasure<Angle> stopperSetpoint = Units.Degrees.of(0).mutableCopy();
}
