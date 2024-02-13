package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Ports;
import frc.robot.lib.Utils;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public class HoodIOReal implements HoodIO {
    private final TalonFX motor;
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Ports.Hood.ENCODER_ID);
    private final PositionVoltage positionControl =
            new PositionVoltage(0).withEnableFOC(true);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public HoodIOReal() {
        motor = new TalonFX(Ports.Hood.MOTOR_ID);
        motor.getConfigurator().apply(HoodConstants.MOTOR_CONFIGURATION);

        LoggedTunableNumber.ifChanged(hashCode(),
                (arr) ->  {
                    motor.getConfigurator().apply(
                            HoodConstants.MOTOR_CONFIGURATION.Slot0.withKP(arr[0]));
                    motor.getConfigurator().apply(
                            HoodConstants.MOTOR_CONFIGURATION.Slot0.withKI(arr[1]));
                    motor.getConfigurator().apply(
                            HoodConstants.MOTOR_CONFIGURATION.Slot0.withKD(arr[2]));
                    motor.getConfigurator().apply(
                            HoodConstants.MOTOR_CONFIGURATION.Slot0.withKS(arr[3]));
                    motor.getConfigurator().apply(
                            HoodConstants.MOTOR_CONFIGURATION.MotionMagic.withMotionMagicExpo_kV(arr[4]));
                    motor.getConfigurator().apply(
                            HoodConstants.MOTOR_CONFIGURATION.MotionMagic.withMotionMagicExpo_kA(arr[5]));
                    motor.getConfigurator().apply(
                            HoodConstants.MOTOR_CONFIGURATION.Slot0.withKG(arr[6]));
                },
                HoodConstants.kP,
                HoodConstants.kI,
                HoodConstants.kD,
                HoodConstants.kS,
                HoodConstants.kV,
                HoodConstants.kA,
                HoodConstants.kG);
    }

    @Override
    public void updateInternalEncoder() {
        motor.setPosition(getEncoderPosition());
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        inputs.angleSetpoint.mut_replace(angle);
        motor.setControl(positionControl.withPosition(angle.in(Units.Rotations)));
    }

    @Override
    public void setPower(double power) {
        inputs.powerSetpoint = power;
        motor.setControl(dutyCycleOut.withOutput(power));
    }

    private double getEncoderPosition() {
        double val = absoluteEncoder.getAbsolutePosition() - HoodConstants.ABSOLUTE_ENCODER_OFFSET.get();
        return Utils.normalize(Rotation2d.fromRotations(val)).getRotations();
    }

    @Override
    public void updateInputs() {
        inputs.angle.mut_replace(motor.getPosition().getValue(), Units.Rotations);
        inputs.absoluteEncoderAngle.mut_replace(
                getEncoderPosition(), Units.Rotations);
        inputs.voltage.mut_replace(motor.getMotorVoltage().getValue(), Units.Volts);
    }
}
