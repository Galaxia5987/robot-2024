package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class GripperIOReal implements GripperIO {
    private final TalonFX angleMotor;
    private final CANSparkMax rollerMotor;
    private final DigitalInput sensor;
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);
    private final MotionMagicExpoTorqueCurrentFOC positionRequest =
            new MotionMagicExpoTorqueCurrentFOC(0);
    private final VoltageOut powerRequest = new VoltageOut(0).withEnableFOC(true);

    private GripperIOReal() {
        angleMotor = new TalonFX(0);
        angleMotor.getConfigurator().apply(GripperConstants.MOTOR_CONFIGURATION);

        rollerMotor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);

        sensor = new DigitalInput(0);

        absoluteEncoder.setPositionOffset(GripperConstants.ABSOLUTE_ENCODER_OFFSET.get());

        rollerMotor.setSmartCurrentLimit(GripperConstants.CURRENT_LIMIT);
        rollerMotor.setInverted(GripperConstants.ROLLER_INVERTED_VALUE);
        rollerMotor.burnFlash();
    }

    @Override
    public void setRollerMotorPower(double power) {
        inputs.rollerPowerSetpoint = power;
        rollerMotor.set(power);
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.setControl(powerRequest.withOutput(power));
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        angleMotor.setControl(
                positionRequest.withPosition(
                        Math.IEEEremainder(new Rotation2d(angle).getRotations(), 180)));
    }

    public boolean hasNote() {
        return sensor.get();
    }

    public void stopWrist() {
        angleMotor.stopMotor();
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    @Override
    public void updateInputs() {
        inputs.angleMotorVoltage.mut_replace(angleMotor.get(), Units.Volts);
        inputs.rollerMotorVoltage.mut_replace(rollerMotor.get(), Units.Volts);
        inputs.currentAngle.mut_replace(angleMotor.getPosition().getValue(), Units.Rotations);
        inputs.hasNote = hasNote();
        inputs.encoderPosition.mut_replace(absoluteEncoder.getAbsolutePosition(), Units.Degrees);
    }
}
