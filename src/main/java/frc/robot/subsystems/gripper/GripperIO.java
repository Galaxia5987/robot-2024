package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;

public interface GripperIO {

    GripperInputsAutoLogged inputs = new GripperInputsAutoLogged();

    void setRollerMotorPower(double power);

    void setAngleMotorPower(double power);

    void setAngle(MutableMeasure<Angle> angle);

    void updateInputs(GripperInputs inputs);
}
