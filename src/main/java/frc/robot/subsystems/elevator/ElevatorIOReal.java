package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Ports;
import frc.robot.lib.math.differential.BooleanTrigger;

public class ElevatorIOReal implements ElevatorIO {
    private final Servo servo;
    private final TalonFX mainMotor;
    private final TalonFX auxMotor;

    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
    private final VoltageOut powerControl = new VoltageOut(0);

    private final DigitalInput sensor = new DigitalInput(1);
    private final BooleanTrigger sensorTrigger = new BooleanTrigger();
    private final BooleanTrigger movingWeightTrigger = new BooleanTrigger();

    private static final MutableMeasure<Mass> movingWeight = ElevatorConstants.HOOKS_MASS;

    public ElevatorIOReal() {
        mainMotor = new TalonFX(Ports.Elevator.MAIN_ID);
        auxMotor = new TalonFX(Ports.Elevator.AUX_ID);
        servo = new Servo(0);

        mainMotor.getConfigurator().apply(ElevatorConstants.MAIN_MOTOR_CONFIGURATION);
        auxMotor.getConfigurator().apply(ElevatorConstants.AUX_MOTOR_CONFIGURATION);

        auxMotor.setControl(new StrictFollower(mainMotor.getDeviceID()));
    }

    @Override
    public void setPower(double power) {
        mainMotor.setControl(powerControl.withOutput(power * 12));
    }

    @Override
    public void setHeight(MutableMeasure<Distance> height) {
        mainMotor.setControl(
                positionControl
                        .withPosition(height.in(Units.Meters))
                        .withFeedForward(
                                Math.signum(
                                                inputs.heightSetpoint
                                                        .minus(inputs.hooksHeight)
                                                        .baseUnitMagnitude())
                                        * ElevatorConstants.KS.get()));
    }

    public void openStopper() {
        inputs.stopperSetpoint = ElevatorConstants.OPEN_POSITION;
        servo.set(ElevatorConstants.OPEN_POSITION.in(Degrees));
    }

    public void closeStopper() {
        inputs.stopperSetpoint = ElevatorConstants.LOCKED_POSITION;
        servo.set(ElevatorConstants.LOCKED_POSITION.in(Degrees));
    }

    public void stopMotor() {
        mainMotor.stopMotor();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.isBottom = !sensor.get();
        sensorTrigger.update(inputs.isBottom);

        inputs.hooksHeight.mut_replace(mainMotor.getPosition().getValue(), Meters);

        inputs.carriageHeight.mut_replace(
                inputs.hooksHeight.gt(ElevatorConstants.GRIPPER_TO_HOOKS)
                        ? inputs.hooksHeight.minus(ElevatorConstants.GRIPPER_TO_HOOKS)
                        : Meters.zero());

        inputs.stopperAngle.mut_replace(servo.getAngle(), Degrees);

        movingWeightTrigger.update(inputs.hooksHeight.gt(ElevatorConstants.GRIPPER_TO_HOOKS));
        if (movingWeightTrigger.triggered()) {
            ElevatorConstants.MAIN_MOTOR_CONFIGURATION.Slot0.kG = -2.5;
            ElevatorConstants.AUX_MOTOR_CONFIGURATION.Slot0.kG = -2.5;
            mainMotor.getConfigurator().apply(ElevatorConstants.MAIN_MOTOR_CONFIGURATION);
            auxMotor.getConfigurator().apply(ElevatorConstants.AUX_MOTOR_CONFIGURATION);
        } else if (movingWeightTrigger.released()) {
            ElevatorConstants.MAIN_MOTOR_CONFIGURATION.Slot0.kG = -9.0;
            ElevatorConstants.AUX_MOTOR_CONFIGURATION.Slot0.kG = -9.0;
            mainMotor.getConfigurator().apply(ElevatorConstants.MAIN_MOTOR_CONFIGURATION);
            auxMotor.getConfigurator().apply(ElevatorConstants.AUX_MOTOR_CONFIGURATION);
        }

        if (sensorTrigger.triggered()) {
            mainMotor.setPosition(0);
            auxMotor.setPosition(0);
        }
    }
}
