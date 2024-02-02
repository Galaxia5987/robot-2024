package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GripperInputs {
    public MutableMeasure<Angle> currentAngle = Units.Radians.of(0).mutableCopy();
    public MutableMeasure<Angle> angleSetpoint = Units.Radians.of(0).mutableCopy();
    public double rollerPowerSetpoint = 0.0;
    public MutableMeasure<Voltage> angleMotorVoltage = Units.Volts.of(0).mutableCopy();
    public MutableMeasure<Voltage> rollerMotorVoltage = Units.Volts.of(0).mutableCopy();
    public boolean hasNote = false;
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration = Units.MetersPerSecondPerSecond.of(0).mutableCopy();
}
