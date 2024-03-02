package frc.robot.subsystems.climb;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimbInputs {
    public MutableMeasure<Distance> hooksHeight = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Distance> carriageHeight = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Distance> heightSetpoint = Units.Meters.of(0).mutableCopy();
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration =
            Units.MetersPerSecondPerSecond.of(0).mutableCopy();
    public ClimbIO.ControlMode controlMode = ClimbIO.ControlMode.POSITION;
    public boolean isBottom = true;
    public MutableMeasure<Angle> stopperAngle = Units.Degrees.of(0).mutableCopy();
    public MutableMeasure<Angle> stopperSetpoint = Units.Degrees.of(0).mutableCopy();
}
