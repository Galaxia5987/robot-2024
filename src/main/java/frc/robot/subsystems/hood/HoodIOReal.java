package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class HoodIOReal implements HoodIO {
    private final TalonFX motor;
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(9); // TODO: to be changed
    private final MotionMagicExpoTorqueCurrentFOC positionControl =
            new MotionMagicExpoTorqueCurrentFOC(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public HoodIOReal() {
        motor = new TalonFX(6); // TODO: to be changed
        motor.getConfigurator().apply(HoodConstants.motorConfiguration);
    }

    @Override
    public void updateInternalEncoder() {
        motor.setPosition(
                absoluteEncoder.getAbsolutePosition()
                        + HoodConstants.ABSOLUTE_ENCODER_OFFSET.in(Units.Rotations));
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        inputs.controlMode = Mode.ANGLE;
        inputs.angleSetpoint.mut_replace(angle);
        motor.setControl(positionControl.withPosition(angle.in(Units.Rotations)));
    }

    @Override
    public void setPower(double power) {
        inputs.controlMode = Mode.POWER;
        inputs.powerSetpoint = power;
        motor.setControl(dutyCycleOut.withOutput(power));
    }

    @Override
    public void updateInputs() {
        inputs.angle.mut_replace(motor.getPosition().getValue(), Units.Rotations);
        inputs.absoluteEncoderAngle.mut_replace(
                absoluteEncoder.getAbsolutePosition(), Units.Rotations);
        inputs.voltage.mut_replace(motor.getMotorVoltage().getValue(), Units.Volts);
    }
}
