package frc.robot.subsystems.hood;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.lib.Utils;

public class HoodIOReal implements HoodIO {
    private final TalonFX angleMotor = new TalonFX(HoodConstants.angleMotorPort);
    private final PositionVoltage positionVoltageOutput = new PositionVoltage(.0);
    private final TalonSRX absoluteEncoder = new TalonSRX(HoodConstants.encoderPort);

    private HoodIOReal() {
        absoluteEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    private double getAbsoluteEncoderAngle(){
        var value =
                absoluteEncoder.getSelectedSensorPosition() % HoodConstants.ENCODER_TICKS_PER_ROTATION
                        / HoodConstants.ENCODER_TICKS_PER_ROTATION - HoodConstants.ABSOLUTE_ENCODER_OFFSET;

        return Utils.normalizeRotations(value).getRotations();
    }
    @Override
    public void updateInputs() {
        inputs.angle = angleMotor.getPosition().getValue() * Constants.toRadiansFromRotation;
        inputs.voltage = angleMotor.getSupplyVoltage().getValue();
//        inputs.absoluteEncoderAngle = absoluteEncoder.
    }

    @Override
    public void setAngle(double angle) {
        angleMotor.setControl(positionVoltageOutput.withPosition(angle));
    }

    @Override
    public void updateInternalEncoder() {

    }
}
