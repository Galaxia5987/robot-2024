package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GripperInputs {
    public MutableMeasure<Angle> currentAngle = Units.Degrees.of(0).mutableCopy();
    public MutableMeasure<Angle> angleSetpoint = Units.Degrees.of(0).mutableCopy();
    public double rollerPowerSetpoint = 0.0;
    public MutableMeasure<Voltage> angleMotorVoltage = Units.Volts.of(0).mutableCopy();
    public MutableMeasure<Voltage> rollerMotorVoltage = Units.Volts.of(0).mutableCopy();
    public boolean hasNote = false;
    public MutableMeasure<Velocity<Velocity<Angle>>> acceleration =
            Units.RotationsPerSecond.per(Units.Second).of(0).mutableCopy();
    public MutableMeasure<Angle> encoderPosition = Units.Degrees.of(0).mutableCopy();
    public MutableMeasure<Angle> noOffsetEncoderPosition = Units.Degrees.of(0).mutableCopy();
}
