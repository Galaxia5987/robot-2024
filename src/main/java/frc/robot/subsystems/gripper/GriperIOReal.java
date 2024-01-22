package frc.robot.subsystems.gripper;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;

public class GriperIOReal implements GripperIO {
    @Override
    public void setSpeedMotorPower(double power) {}

    @Override
    public void setAngleMotorPower(double power) {}

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {}

    @Override
    public void updateInputs(GripperInputs inputs) {}
}