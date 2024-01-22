package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GripperInputs {
    public MutableMeasure<Angle> currentAngle;
    public MutableMeasure<Angle> angleSetpoint;
    public double rollerSpeed;
    public MutableMeasure<Voltage> angleMotorVoltage;
    public MutableMeasure<Voltage> spinMotorVoltage;
    public boolean hasNote;
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration;
}
