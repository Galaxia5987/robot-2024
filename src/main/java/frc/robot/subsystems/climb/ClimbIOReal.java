package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Ports;
import frc.robot.lib.math.differential.BooleanTrigger;

public class ClimbIOReal implements ClimbIO {
    private final Servo servo;
    private final TalonFX mainMotor;
    private final TalonFX auxMotor;

    private final DutyCycleOut powerControl = new DutyCycleOut(0).withEnableFOC(true);
    private final PositionVoltage positionControl = new PositionVoltage(0).withEnableFOC(true);

    public ClimbIOReal() {
        mainMotor = new TalonFX(Ports.Elevator.MAIN_ID);
        auxMotor = new TalonFX(Ports.Elevator.AUX_ID);
        servo = new Servo(9);

        mainMotor.getConfigurator().apply(ClimbConstants.MAIN_MOTOR_CONFIGURATION);
        auxMotor.getConfigurator().apply(ClimbConstants.AUX_MOTOR_CONFIGURATION);

        auxMotor.setControl(new StrictFollower(mainMotor.getDeviceID()));
    }

    @Override
    public void setPower(double power) {
        mainMotor.setControl(powerControl.withOutput(power));
    }

    public void openStopper() {
        inputs.stopperSetpoint = ClimbConstants.OPEN_POSITION;
        servo.set(ClimbConstants.OPEN_POSITION.in(Units.Degrees));
    }

    public void closeStopper() {
        inputs.stopperSetpoint = ClimbConstants.LOCKED_POSITION;
        servo.set(ClimbConstants.LOCKED_POSITION.in(Units.Degrees));
    }

    public void stopMotor() {
        mainMotor.stopMotor();
    }

    public void manualReset() {
        mainMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbInputs inputs) {
        inputs.stopperAngle.mut_replace(servo.getAngle(), Units.Degrees);
    }
}
