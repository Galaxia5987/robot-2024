package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;

public class GripperIOReal implements GripperIO {
    private final TalonFX angleMotor;
    private final CANSparkMax rollerMotor;
    private final SparkLimitSwitch forwardLimitSwitch;
    private final SparkLimitSwitch reverseLimitSwitch;
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Ports.Gripper.ENCODER_ID);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut powerRequest = new VoltageOut(0).withEnableFOC(true);
    private final DigitalInput sensor = new DigitalInput(4);
    private final Timer timer = new Timer();

    public GripperIOReal() {
        angleMotor = new TalonFX(Ports.Gripper.ANGLE_ID);
        angleMotor.getConfigurator().apply(GripperConstants.MOTOR_CONFIGURATION);

        rollerMotor =
                new CANSparkMax(Ports.Gripper.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setSmartCurrentLimit(40);
        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        forwardLimitSwitch =
                rollerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        reverseLimitSwitch =
                rollerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        forwardLimitSwitch.enableLimitSwitch(false);
        reverseLimitSwitch.enableLimitSwitch(false);
        rollerMotor.burnFlash();

        rollerMotor.setSmartCurrentLimit(GripperConstants.CURRENT_LIMIT);
        rollerMotor.setInverted(GripperConstants.ROLLER_INVERTED_VALUE);
        rollerMotor.burnFlash();

        timer.start();
        timer.reset();
    }

    @Override
    public void setRollerMotorPower(double power) {
        inputs.rollerPowerSetpoint = power;
        System.out.println("Set roller power powered with: " + power);
        rollerMotor.set(power);
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.setControl(powerRequest.withOutput(power));
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        inputs.angleSetpoint.mut_replace(angle);
        angleMotor.setControl(
                positionRequest.withPosition(
                        Math.IEEEremainder(new Rotation2d(angle).getRotations(), 1)));
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    @Override
    public void updateInputs() {
        inputs.angleMotorVoltage.mut_replace(
                angleMotor.get() * RobotController.getBatteryVoltage(), Units.Volts);
        inputs.rollerMotorVoltage.mut_replace(
                rollerMotor.get() * RobotController.getBatteryVoltage(), Units.Volts);
        inputs.currentAngle.mut_replace(angleMotor.getPosition().getValue(), Units.Rotations);
        inputs.hasNote = hasNote();
        inputs.noOffsetEncoderPosition.mut_replace(
                -absoluteEncoder.getAbsolutePosition(), Units.Rotations);
        inputs.encoderPosition.mut_replace(
                inputs.noOffsetEncoderPosition.in(Units.Rotations)
                        - GripperConstants.ABSOLUTE_ENCODER_OFFSET.get(),
                Units.Rotations);
        inputs.encoderPosition.mut_replace(
                Math.IEEEremainder(inputs.encoderPosition.in(Units.Rotations), 1), Units.Rotations);

        if (timer.advanceIfElapsed(2)) {
            angleMotor.setPosition(inputs.encoderPosition.in(Units.Rotations));
        }
    }
}
