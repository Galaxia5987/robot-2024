package frc.robot.subsystems.gripper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.gripper.GripperConstants.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import lib.Utils;
import lib.motors.TalonFXSim;

public class GripperIOSim implements GripperIO {
    private final TalonFXSim rollerMotor;
    private final TalonFXSim angleMotor;

    private final DutyCycleOut powerRequest = new DutyCycleOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public static PIDController angleController = new PIDController(KP.get(), KI.get(), KD.get());

    public GripperIOSim() {
        angleMotor =
                new TalonFXSim(
                        1,
                        GripperConstants.ANGLE_MOTOR_GEAR_RATIO,
                        0.000_01,
                        GripperConstants.ANGLE_MOTOR_GEAR_RATIO);

        rollerMotor = new TalonFXSim(1, 1, 0.5, 1);

        angleMotor.setController(angleController);
    }

    @Override
    public void setRollerMotorPower(double power) {
        rollerMotor.setControl(powerRequest.withOutput(power));
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.setControl(powerRequest.withOutput(power));
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        inputs.angleSetpoint = angle;

        angleMotor.setControl(
                positionRequest.withPosition(angle.mut_replace(Utils.normalize(angle.in(Radians)), Radians).in(Rotations)));
    }

    @Override
    public void updateInputs() {
        rollerMotor.update(Timer.getFPGATimestamp());
        angleMotor.update(Timer.getFPGATimestamp());
        inputs.currentAngle = Rotations.of(angleMotor.getPosition()).mutableCopy();
    }
}
