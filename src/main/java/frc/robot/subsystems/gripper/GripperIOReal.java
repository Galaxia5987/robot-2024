package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;

public class GripperIOReal implements GripperIO {
    private final TalonFX angleMotor;
    private final CANSparkMax rollerMotor;
    private final SparkLimitSwitch forwardLimitSwitch;
    private final SparkLimitSwitch reverseLimitSwitch;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut powerRequest = new DutyCycleOut(0).withEnableFOC(true);
    private final Timer timer = new Timer();
    private final Debouncer debouncer = new Debouncer(0.0);

    public GripperIOReal() {
        angleMotor = new TalonFX(Ports.Gripper.ANGLE_ID);
        angleMotor.getConfigurator().apply(GripperConstants.MOTOR_CONFIGURATION);

        rollerMotor =
                new CANSparkMax(Ports.Gripper.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setSmartCurrentLimit(40);
        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        forwardLimitSwitch = rollerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        reverseLimitSwitch = rollerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        forwardLimitSwitch.enableLimitSwitch(false);
        reverseLimitSwitch.enableLimitSwitch(false);
        rollerMotor.burnFlash();

        rollerMotor.setSmartCurrentLimit(GripperConstants.CURRENT_LIMIT);
        rollerMotor.setInverted(GripperConstants.ROLLER_INVERTED_VALUE);
        rollerMotor.burnFlash();

        angleMotor.getSupplyVoltage().setUpdateFrequency(100);

        timer.start();
        timer.reset();
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
        inputs.angleSetpoint.mut_replace(angle);
        angleMotor.setControl(
                positionRequest.withPosition(
                        Math.IEEEremainder(new Rotation2d(angle).getRotations(), 1)));
    }

    public boolean hasNote() {
        return debouncer.calculate(
                forwardLimitSwitch.isPressed() || reverseLimitSwitch.isPressed());
    }

    @Override
    public void updateInputs() {
        inputs.angleMotorVoltage.mut_replace(angleMotor.getSupplyVoltage().getValue(), Units.Volts);
        inputs.rollerMotorVoltage.mut_replace(
                rollerMotor.get() * RobotController.getBatteryVoltage(), Units.Volts);
        inputs.currentAngle.mut_replace(angleMotor.getPosition().getValue(), Units.Rotations);
        inputs.hasNote = hasNote();
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
