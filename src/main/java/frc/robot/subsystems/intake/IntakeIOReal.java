package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {

    private final CANSparkMax spinMotor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax centerMotor =
            new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(21);
    private final MotionMagicExpoTorqueCurrentFOC positionRequest =
            new MotionMagicExpoTorqueCurrentFOC(0);

    public IntakeIOReal() {

        angleMotor.getConfigurator().apply(IntakeConstants.ANGLE_CONFIGURATION.Slot0);

        centerMotor.restoreFactoryDefaults();
        centerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        centerMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        centerMotor.setInverted(true);
        spinMotor.restoreFactoryDefaults();
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        spinMotor.setInverted(true);
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
    public void setRollerSpeed(double speed) {
        spinMotor.set(speed);
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
        spinMotor.burnFlash();
        centerMotor.burnFlash();
    }
}
