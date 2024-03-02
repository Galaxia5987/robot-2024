package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Ports;
import frc.robot.lib.math.differential.BooleanTrigger;

public class ClimbIOReal implements ClimbIO {
    private final Servo servo;
    private final TalonFX mainMotor;
    private final TalonFX auxMotor;

    private final PositionVoltage positionControl = new PositionVoltage(0).withEnableFOC(true);
    private final DutyCycleOut powerControl = new DutyCycleOut(0).withEnableFOC(true);

    private final DigitalInput sensor = new DigitalInput(1);
    private final BooleanTrigger sensorTrigger = new BooleanTrigger();
    private final BooleanTrigger movingWeightTrigger = new BooleanTrigger();

    private double kS = ClimbConstants.KS_FIRST_STAGE.get();
    private double kG = ClimbConstants.KG_FIRST_STAGE.get();

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

    @Override
    public void setHeight(MutableMeasure<Distance> height) {
        double feedforward =
                Math.signum(inputs.heightSetpoint.minus(inputs.hooksHeight).baseUnitMagnitude())
                                * kS
                        + kG;
        mainMotor.setControl(
                positionControl.withPosition(height.in(Units.Meters)).withFeedForward(feedforward));
    }

    public void openStopper() {
        inputs.stopperSetpoint = ClimbConstants.OPEN_POSITION;
        servo.set(ClimbConstants.OPEN_POSITION.in(Degrees));
    }

    public void closeStopper() {
        inputs.stopperSetpoint = ClimbConstants.LOCKED_POSITION;
        servo.set(ClimbConstants.LOCKED_POSITION.in(Degrees));
    }

    public void stopMotor() {
        mainMotor.stopMotor();
    }

    public void manualReset() {
        mainMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbInputs inputs) {
        inputs.isBottom = !sensor.get();
        sensorTrigger.update(inputs.isBottom);

        inputs.hooksHeight.mut_replace(mainMotor.getPosition().getValue(), Meters);

        inputs.carriageHeight.mut_replace(
                inputs.hooksHeight.gt(ClimbConstants.GRIPPER_TO_HOOKS)
                        ? inputs.hooksHeight.minus(ClimbConstants.GRIPPER_TO_HOOKS)
                        : Meters.zero());

        inputs.stopperAngle.mut_replace(servo.getAngle(), Degrees);

        movingWeightTrigger.update(inputs.hooksHeight.gt(ClimbConstants.GRIPPER_TO_HOOKS));
        if (movingWeightTrigger.triggered()) {
            kG = ClimbConstants.KG_SECOND_STAGE.get();
            kS = ClimbConstants.KS_SECOND_STAGE.get();
        } else if (movingWeightTrigger.released()) {
            kG = ClimbConstants.KG_FIRST_STAGE.get();
            kS = ClimbConstants.KS_FIRST_STAGE.get();
        }

        if (sensorTrigger.triggered()) {
            mainMotor.setPosition(0);
            auxMotor.setPosition(0);
        }
    }
}
