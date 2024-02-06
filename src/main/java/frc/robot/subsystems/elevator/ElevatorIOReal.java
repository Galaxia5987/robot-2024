package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Ports;

public class ElevatorIOReal implements ElevatorIO {
    private final Servo servo;
    private final TalonFX mainMotor;
    private final TalonFX auxMotor;

    private final DigitalInput sensor;

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
    private final DutyCycleOut powerControl = new DutyCycleOut(0);

    public ElevatorIOReal() {
        mainMotor = new TalonFX(Ports.Elevator.MAIN_MOTOR);
        auxMotor = new TalonFX(Ports.Elevator.AUX_MOTOR);
        servo = new Servo(Ports.Elevator.SERVO);
        sensor = new DigitalInput(Ports.Elevator.LIMIT_SWITCH);

        mainMotor.getConfigurator().apply(ElevatorConstants.MAIN_MOTOR_CONFIGURATION);
        auxMotor.getConfigurator().apply(ElevatorConstants.AUX_MOTOR_CONFIGURATION);

        auxMotor.setControl(new StrictFollower(mainMotor.getDeviceID()));
    }

    @Override
    public void setPower(double power) {
        mainMotor.setControl(powerControl.withOutput(power));
    }

    @Override
    public void setHeight(MutableMeasure<Distance> height) {
        mainMotor.setControl(positionControl.withPosition(height.in(Units.Meters)));
    }

    public boolean atBottom() {
        return sensor.get();
    }

    public void openStopper(){
        inputs.stopperSetpoint = ElevatorConstants.STOPPER_OPEN;
        servo.set(ElevatorConstants.STOPPER_OPEN.in(Degrees));
    }
    public void closeStopper(){
        inputs.stopperSetpoint = ElevatorConstants.STOPPER_CLOSE;
        servo.set(ElevatorConstants.STOPPER_CLOSE.in(Degrees));
    }

    public void resetEncoder() {
        if (atBottom()) {
            setHeight((ElevatorConstants.STARTING_HEIGHT).mutableCopy());
        }
    }

    public void stopMotor() {
        mainMotor.stopMotor();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.isBottom = atBottom();
        inputs.carriageHeight.mut_replace(mainMotor.getPosition().getValue(), Meters);

        inputs.gripperHeight.mut_replace(
                inputs.carriageHeight.gt(ElevatorConstants.GRIPPER_HEIGHT)
                        ? inputs.carriageHeight
                                .mutableCopy()
                                .mut_minus(ElevatorConstants.GRIPPER_HEIGHT)
                        : Meters.zero());

        inputs.stopperAngle.mut_replace(servo.getAngle(), Degrees);
    }
}
