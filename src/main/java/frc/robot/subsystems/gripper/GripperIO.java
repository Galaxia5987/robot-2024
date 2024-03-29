package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;

public interface GripperIO {

    GripperInputsAutoLogged inputs = new GripperInputsAutoLogged();

    default void setRollerMotorPower(double power) {}

    default void updateInputs() {}
}
