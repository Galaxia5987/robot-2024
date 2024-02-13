package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {

    public final CANSparkMax spinMotor = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax centerMotor =
            new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(1);
    private final MotionMagicExpoTorqueCurrentFOC positionRequest =
            new MotionMagicExpoTorqueCurrentFOC(0);
    private SimpleMotorFeedforward spinMotorFeedforward;

    public IntakeIOReal() {

        angleMotor.getConfigurator().apply(ANGLE_CONFIGURATION.Slot0);

        centerMotor.restoreFactoryDefaults();
        centerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        centerMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE.in(Units.Volts));
        centerMotor.setInverted(true);
        spinMotor.restoreFactoryDefaults();
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE.in(Units.Volts));
        spinMotor.setInverted(true);
        spinMotor.getPIDController().setP(SPIN_KP.get());
        spinMotor.getPIDController().setI(SPIN_KI.get());
        spinMotor.getPIDController().setD(SPIN_KD.get());
        spinMotorFeedforward =
                new SimpleMotorFeedforward(SPIN_KS.get(), SPIN_KV.get(), SPIN_KA.get());
        for (int i = 1; i <= 6; i++) {
            spinMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.fromId(i), 50);
            centerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.fromId(i), 50);
        }
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
    public void updateInputs() {
        inputs.currentAngle.mut_replace((angleMotor.getPosition().getValue()), Units.Degrees);
        inputs.currentCenterRollerSpeed.mut_replace(
                (centerMotor.getEncoder().getVelocity()), Units.RotationsPerSecond);
        inputs.angleMotorVoltage.mut_replace(
                (angleMotor.getMotorVoltage().getValue()), Units.Volts);
        inputs.spinMotorVoltage.mut_replace((spinMotor.getBusVoltage()), Units.Volts);
        inputs.centerMotorVoltage.mut_replace((centerMotor.getBusVoltage()), Units.Volts);
        inputs.currentRollerSpeed.mut_replace(spinMotor.getEncoder().getVelocity(), Units.RPM);
    }
}
