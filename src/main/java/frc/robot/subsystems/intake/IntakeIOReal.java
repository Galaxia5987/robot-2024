package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Ports;

public class IntakeIOReal implements IntakeIO {

    public final CANSparkMax spinMotor =
            new CANSparkMax(Ports.Intake.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax centerMotor =
            new CANSparkMax(Ports.Intake.CENTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(Ports.Intake.ANGLE_ID);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
    private final SimpleMotorFeedforward spinMotorFeedforward;

    public IntakeIOReal() {
        angleMotor.getConfigurator().apply(ANGLE_CONFIGURATION);

        centerMotor.restoreFactoryDefaults();
        centerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        centerMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE.in(Units.Volts));
        centerMotor.setInverted(false);
        centerMotor.setSmartCurrentLimit(CENTER_CURRENT_LIMIT);
        spinMotor.restoreFactoryDefaults();
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE.in(Units.Volts));
        spinMotor.setInverted(false);
        spinMotor.getPIDController().setP(SPIN_KP.get());
        spinMotor.getPIDController().setI(SPIN_KI.get());
        spinMotor.getPIDController().setD(SPIN_KD.get());
        spinMotor.setSmartCurrentLimit(SPIN_CURRENT_LIMIT);
        spinMotorFeedforward =
                new SimpleMotorFeedforward(SPIN_KS.get(), SPIN_KV.get(), SPIN_KA.get());
        spinMotor.burnFlash();
        centerMotor.burnFlash();
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        angleMotor.setControl(positionRequest.withPosition(angle.in(Units.Degrees)));
    }

    @Override
    public void setRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        spinMotor
                .getPIDController()
                .setReference(
                        speed.in(Units.RotationsPerSecond),
                        CANSparkBase.ControlType.kVelocity,
                        0,
                        spinMotorFeedforward.calculate(speed.in(Units.RotationsPerSecond)));
    }

    @Override
    public void setCenterRollerSpeed(double speed) {
        centerMotor.set(speed);
    }

    @Override
    public void reset(Measure<Angle> angle) {
        angleMotor.setPosition(angle.in(Units.Degrees));
    }

    @Override
    public void updateInputs() {
        inputs.currentAngle.mut_replace((angleMotor.getPosition().getValue()), Units.Degrees);
        inputs.currentCenterRollerSpeed.mut_replace(
                (centerMotor.getEncoder().getVelocity()), Units.RotationsPerSecond);
        inputs.angleMotorVoltage.mut_replace(
                (angleMotor.getMotorVoltage().getValue()), Units.Volts);
        inputs.spinMotorVoltage.mut_replace(
                (spinMotor.getAppliedOutput() * RobotController.getBatteryVoltage()), Units.Volts);
        inputs.centerMotorVoltage.mut_replace(
                (centerMotor.getAppliedOutput() * RobotController.getBatteryVoltage()),
                Units.Volts);
        inputs.currentRollerSpeed.mut_replace(spinMotor.getEncoder().getVelocity(), Units.RPM);
    }
}
