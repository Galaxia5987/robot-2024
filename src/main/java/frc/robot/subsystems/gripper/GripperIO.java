package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;

public interface GripperIO {

    GripperInputs inputs = new GripperInputs();

    void setSpeedMotorPower(double power);

    void setAngleMotorPower(double power);

    void setAngle(MutableMeasure<Angle> angle);

    void updateInputs(GripperInputs inputs);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }
}
