package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.motors.SparkMaxSim;
import frc.robot.lib.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private final TalonFXSim angleMotor;
    private final SparkMaxSim frontRoller;
    private final SparkMaxSim centerRoller;
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    public static PIDController angleController =
            new PIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get());

    public IntakeIOSim() {
        angleMotor =
                new TalonFXSim(
                        1, IntakeConstants.GEAR_RATIO, 0.003, 360 * IntakeConstants.GEAR_RATIO);
        frontRoller = new SparkMaxSim(1, 1, 0.0003, 1);
        centerRoller = new SparkMaxSim(1, 1, 0.0003, 1);
        angleMotor.setController(angleController);
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        inputs.angleSetpoint = angle;
        angleMotor.setControl(positionRequest.withPosition(angle.in(Units.Degrees)));
    }

    @Override
    public void setRollerSpeed(double speed) {
        frontRoller.set(speed);
    }

    @Override
    public void setCenterRollerSpeed(double speed) {
        centerRoller.setReference(speed, CANSparkBase.ControlType.kDutyCycle);
    }

    @Override
    public void reset(Measure<Angle> angle) {}

    @Override
    public void setAnglePower(double power) {
        angleMotor.setControl(new DutyCycleOut(power));
    }

    @Override
    public void updateInputs() {
        angleMotor.update(Timer.getFPGATimestamp());
        centerRoller.update(Timer.getFPGATimestamp());
        frontRoller.update(Timer.getFPGATimestamp());
        inputs.currentAngle.mut_replace((angleMotor.getPosition()), Units.Degrees);
        inputs.angleMotorVoltage.mut_replace(angleMotor.getAppliedVoltage(), Units.Volts);
        inputs.spinMotorVoltage.mut_replace(frontRoller.getBusVoltage(), Units.Volts);
        inputs.centerMotorVoltage.mut_replace(centerRoller.getBusVoltage(), Units.Volts);
    }
}
