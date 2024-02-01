package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.elevator.ElevatorConstants;
import lib.motors.TalonFXSim;

import static edu.wpi.first.units.Units.Radians;

public class GripperIOSim implements GripperIO {
    private final TalonFXSim rollerMotor;
    private final TalonFXSim angleMotor;

    private final DutyCycleOut powerRequest = new DutyCycleOut(0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    public GripperIOSim() {
        angleMotor = new TalonFXSim(1, GripperConstants.ANGLE_MOTOR_GEAR_RATIO, 0.5, GripperConstants.ANGLE_MOTOR_GEAR_RATIO);

        rollerMotor = new TalonFXSim(1, GripperConstants.ROLLER_MOTOR_GEAR_RATIO, 0.5, GripperConstants.ROLLER_MOTOR_GEAR_RATIO);

        angleMotor.setController(new PIDController(ElevatorConstants.KP.get(), ElevatorConstants.KI.get(), ElevatorConstants.KD.get(), 0.02));
    }

    @Override
    public void setRollerMotorPower(double power) {
        powerRequest.withOutput(power);
        rollerMotor.setControl(powerRequest);
    }

    @Override
    public void setAngleMotorPower(double power) {
        powerRequest.withOutput(power);
        angleMotor.setControl(powerRequest);
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        positionRequest.withPosition(angle.in(Radians));
        angleMotor.setControl(positionRequest);
    }

    @Override
    public void updateInputs() {
        rollerMotor.update(Timer.getFPGATimestamp());
        angleMotor.update(Timer.getFPGATimestamp());
        inputs.currentAngle = Radians.
                of(angleMotor.getPosition()).mutableCopy();
    }
}
