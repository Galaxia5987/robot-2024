package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Ports;

public class IntakeIOReal implements IntakeIO {

    private final CANSparkMax spinMotor =
            new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax centerMotor =
            new CANSparkMax(
                    0, CANSparkLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(21);
    private final TalonFXConfigurator angleConfigurator;
    private final MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    private TalonFXConfiguration angleConfiguration = new TalonFXConfiguration();

    public IntakeIOReal() {
        angleConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.ANGLE_GEAR_RATIO;

        angleConfiguration.Feedback.RotorToSensorRatio = 1;

        angleConfigurator = angleMotor.getConfigurator();

        angleConfigurator.apply(angleConfiguration.Slot0);

        spinMotor.restoreFactoryDefaults();
        centerMotor.restoreFactoryDefaults();
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        centerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        centerMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        spinMotor.setInverted(true);
        centerMotor.setInverted(true);

        for (int i = 1; i <= 6; i++) {

            spinMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.fromId(i), 50_000);
            centerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.fromId(i), 50_000);
        }

        spinMotor.burnFlash();
        centerMotor.burnFlash();
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        angleMotor.setControl(
                positionRequest.withPosition(angle.in(Units.Degrees)));
    }

    @Override
    public void setRollerSpeed(double speed) {
        spinMotor.set(speed);
    }

    @Override
    public void setCenterRollerSpeed(double speed) {
        centerMotor.set(speed);
    }

    @Override
    public void updateInputs() {
        inputs.currentAngle.mut_replace(
                Units.Degrees.of(angleMotor.getPosition().getDataCopy().value).mutableCopy());
        inputs.currentCenterRollerSpeed.mut_replace(
                Units.RotationsPerSecond.of(centerMotor.getEncoder().getVelocity()).mutableCopy());
        inputs.angleMotorVoltage.mut_replace(
                Units.Volts.of(angleMotor.getMotorVoltage().getValue()).mutableCopy());
        inputs.spinMotorVoltage.mut_replace(
                Units.Volts.of(spinMotor.getBusVoltage()).mutableCopy());
        inputs.centerMotorVoltage.mut_replace(
                Units.Volts.of(centerMotor.getBusVoltage()).mutableCopy());
    }
}
