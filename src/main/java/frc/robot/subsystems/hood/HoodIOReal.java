package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Ports;
import lib.Utils;

public class HoodIOReal implements HoodIO {
    private final TalonFX motor;
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Ports.Hood.ENCODER_ID);
    private final MotionMagicExpoTorqueCurrentFOC positionControl =
            new MotionMagicExpoTorqueCurrentFOC(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public HoodIOReal() {
        motor = new TalonFX(Ports.Hood.MOTOR_ID);
        motor.getConfigurator().apply(HoodConstants.MOTOR_CONFIGURATION);
    }

    @Override
    public void updateInternalEncoder() {
        motor.setPosition(absoluteEncoder.getAbsolutePosition());
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
