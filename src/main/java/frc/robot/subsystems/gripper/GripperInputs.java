package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GripperInputs {
    public double rollerPowerSetpoint = 0.0;
    public MutableMeasure<Voltage> rollerMotorVoltage = Units.Volts.of(0).mutableCopy();
    public boolean hasNote = false;
}
