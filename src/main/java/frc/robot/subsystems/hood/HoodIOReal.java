package frc.robot.subsystems.hood;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import frc.robot.Ports;
import frc.robot.lib.Utils;

public class HoodIOReal implements HoodIO {
    private final TalonFX motor;
    private final TalonSRX encoder = new TalonSRX(Ports.Hood.ENCODER_ID);
    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);

    public HoodIOReal() {
        motor = new TalonFX(Ports.Hood.MOTOR_ID);
        motor.getConfigurator().apply(HoodConstants.MOTOR_CONFIGURATION);

        encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    @Override
    public void updateInternalEncoder() {
        motor.setPosition(getEncoderPosition());
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        setAngle(angle, 0);
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle, double torqueChassisCompensation) {
        var error = angle.minus(inputs.absoluteEncoderAngle);
        motor.setControl(
                positionControl
                        .withPosition(inputs.internalAngle.plus(error).in(Units.Rotations))
                        .withFeedForward(
                                Math.signum(
                                                        inputs.angleSetpoint
                                                                .minus(inputs.internalAngle)
                                                                .in(Units.Degrees))
                                                * HoodConstants.kS.get()
                                        + torqueChassisCompensation
                                                * HoodConstants.TORQUE_TO_CURRENT));
    }

    private double getEncoderPosition() {
        double val =
                ((encoder.getSelectedSensorPosition() % HoodConstants.ENCODER_TICKS_PER_REVOLUTION)
                                / HoodConstants.ENCODER_TICKS_PER_REVOLUTION)
                        - HoodConstants.ABSOLUTE_ENCODER_OFFSET.get();
        return Utils.normalize(Rotation2d.fromRotations(val)).getRotations();
    }

    @Override
    public void updateInputs() {
        inputs.internalAngle.mut_replace(motor.getPosition().getValue(), Units.Rotations);
        inputs.absoluteEncoderAngle.mut_replace(getEncoderPosition(), Units.Rotations);
        inputs.voltage.mut_replace(motor.getMotorVoltage().getValue(), Units.Volts);
        inputs.absoluteEncoderAngleNoOffset.mut_replace(
                ((encoder.getSelectedSensorPosition() % HoodConstants.ENCODER_TICKS_PER_REVOLUTION)
                        / HoodConstants.ENCODER_TICKS_PER_REVOLUTION),
                Units.Rotations);
    }
}
