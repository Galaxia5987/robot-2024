package frc.robot.subsystems.climb;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimbInputs {
    public MutableMeasure<Angle> stopperAngle = Units.Degrees.of(0).mutableCopy();
    public MutableMeasure<Angle> stopperSetpoint = Units.Degrees.of(0).mutableCopy();
    public MutableMeasure<Voltage> appliedVoltage = Units.Volts.zero().mutableCopy();
}
