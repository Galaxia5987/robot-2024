package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;

public class GripperIOReal implements GripperIO {
    private final CANSparkMax rollerMotor;
    private final Timer timer = new Timer();
    private final DigitalInput sensor = new DigitalInput(2);

    public GripperIOReal() {
        rollerMotor =
                new CANSparkMax(Ports.Gripper.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setSmartCurrentLimit(40);
        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rollerMotor
                .getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
                .enableLimitSwitch(false);
        rollerMotor
                .getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
                .enableLimitSwitch(false);
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
        rollerMotor.set(power);
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    @Override
    public void updateInputs() {
        inputs.rollerMotorVoltage.mut_replace(
                rollerMotor.get() * RobotController.getBatteryVoltage(), Units.Volts);
        inputs.hasNote = hasNote();
    }
}
