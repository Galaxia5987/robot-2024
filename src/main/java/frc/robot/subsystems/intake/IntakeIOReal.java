package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX angleMotor = new TalonFX(IntakeConstants.angleMotorDeviceID);
    private final TalonFX spinMotor = new TalonFX(IntakeConstants.spinMotorDeviceID);
    private final VoltageOut voltageOutput = new VoltageOut(.0);
    private final PositionVoltage positionVoltageOutput = new PositionVoltage(.0);

    @Override
    public void updateInput() {
        inputs.angle = angleMotor.getPosition().getValue() * Constants.toRadiansFromRotation;
        inputs.spinMotorVoltage = spinMotor.getSupplyVoltage().getValue();
        inputs.angleMotorVoltage = angleMotor.getSupplyVoltage().getValue();
    }

    @Override
    public void setAngle(double angle) {
        angleMotor.setControl(positionVoltageOutput.withPosition(angle));
    }

    @Override
    public void setVoltage(double voltage) {
        spinMotor.setControl(voltageOutput.withOutput(voltage));
    }
}
