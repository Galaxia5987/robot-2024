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

public class ElevatorIOReal implements ElevatorIO {
    private final Servo servo;
    private final TalonFX mainMotor;
    private final TalonFX auxMotor;

    private final DigitalInput bottomSensor;
    private final DigitalInput topSensor;

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
    private final DutyCycleOut powerControl = new DutyCycleOut(0);

    public ElevatorIOReal() {
        mainMotor = new TalonFX(0);
        auxMotor = new TalonFX(0);
        servo = new Servo(0);
        bottomSensor = new DigitalInput(0);
        topSensor = new DigitalInput(0);

        mainMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIGURATION);
        auxMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIGURATION);

        auxMotor.setControl(new StrictFollower(mainMotor.getDeviceID()));
    }

    @Override
    public void setPower(double power) {
        mainMotor.setControl(powerControl.withOutput(power));
    }

    @Override
    public void setHeight(MutableMeasure<Distance> height) {
        inputs.heightSetpoint.mut_replace(height);
        mainMotor.setControl(positionControl.withPosition(height.in(Units.Meters)));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.isBottom = atBottom();
        inputs.isTop = atTop();
        inputs.carriageHeight = Meters.of(mainMotor.getPosition().getValue()).mutableCopy();

        inputs.gripperHeight.mut_replace(
                inputs.carriageHeight.gt(ElevatorConstants.GRIPPER_HEIGHT)
                        ? inputs.carriageHeight
                                .mutableCopy()
                                .mut_minus(ElevatorConstants.GRIPPER_HEIGHT)
                        : Meters.zero());

        inputs.servoAngle = Units.Degrees.of(servo.getAngle()).mutableCopy();
    }

    public boolean atTop() {
        return topSensor.get();
    }

    public boolean atBottom() {
        return bottomSensor.get();
    }

    public void flipServo() {
        if (servo.get() == ElevatorConstants.SERVO_OPEN.in(Degrees)) {
            inputs.servoSetpoint = ElevatorConstants.SERVO_CLOSE;
            servo.set(ElevatorConstants.SERVO_CLOSE.in(Degrees));
        } else {
            inputs.servoSetpoint = ElevatorConstants.SERVO_OPEN;
            servo.set(ElevatorConstants.SERVO_OPEN.in(Degrees));
        }
    }

    public void resetEncoder() {
        if (atTop()) {
            setHeight(Meters.of(0).mutableCopy());
        } else if (atTop())
            setHeight(Meters.of(ElevatorConstants.MAX_HEIGHT.in(Meters)).mutableCopy());
    }

    public void stopMotor() {
        mainMotor.setControl(powerControl.withOutput(0));
    }
}
