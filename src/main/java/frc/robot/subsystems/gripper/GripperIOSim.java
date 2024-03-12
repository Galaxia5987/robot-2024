package frc.robot.subsystems.gripper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.gripper.GripperConstants.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.motors.TalonFXSim;

public class GripperIOSim implements GripperIO {
    private final TalonFXSim rollerMotor;

    private final DutyCycleOut powerRequestRoller = new DutyCycleOut(0);

    public GripperIOSim() {
        rollerMotor = new TalonFXSim(1, 1, 0.5, 1);
        inputs.hasNote = true;
    }

    @Override
    public void setRollerMotorPower(double power) {
        rollerMotor.setControl(powerRequestRoller.withOutput(power));
    }

    @Override
    public void updateInputs() {
        rollerMotor.update(Timer.getFPGATimestamp());
        inputs.rollerMotorVoltage.mut_replace(rollerMotor.getAppliedVoltage(), Volts);
    }
}
