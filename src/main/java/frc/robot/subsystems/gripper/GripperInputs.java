package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GripperInputs {
    public MutableMeasure<Angle> currentAngle;
    public MutableMeasure<Angle> angleSetpoint;
    public double rollerPowerSetpoint;
    public MutableMeasure<Voltage> angleMotorVoltage;
    public MutableMeasure<Voltage> rollerMotorVoltage;
    public boolean hasNote;
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration;
    public boolean atSetpoint;
}
