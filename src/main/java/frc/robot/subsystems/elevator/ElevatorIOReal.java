package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.lib.Utils;

public class ElevatorIOReal implements ElevatorIO {
    private final Servo servo;
    private final TalonFX mainMotor;
    private final TalonFX auxMotor;

    private final MotionMagicExpoTorqueCurrentFOC positionControl =
            new MotionMagicExpoTorqueCurrentFOC(0);
    private final DutyCycleOut powerControl = new DutyCycleOut(0);

    private static final MutableMeasure<Mass> movingWeight = ElevatorConstants.HOOKS_MASS;

    private double lastKg = 0;
    private double lastAuxKg = 0;

    public ElevatorIOReal() {
        mainMotor = new TalonFX(0);
        auxMotor = new TalonFX(0);
        servo = new Servo(0);

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
        inputs.isBottom = mainMotor.getFault_ReverseHardLimit().getValue();
        inputs.hooksHeight.mut_replace(mainMotor.getPosition().getValue(), Meters);

        inputs.carriageHeight.mut_replace(
                inputs.hooksHeight.gt(ElevatorConstants.GRIPPER_TO_HOOKS)
                        ? inputs.hooksHeight.mut_minus(ElevatorConstants.GRIPPER_TO_HOOKS)
                        : Meters.zero());

        inputs.stopperAngle.mut_replace(servo.getAngle(), Degrees);

        movingWeight.mut_replace(ElevatorConstants.HOOKS_MASS);
        if (inputs.hooksHeight.gt(ElevatorConstants.GRIPPER_TO_HOOKS)) {
            movingWeight.mut_acc(ElevatorConstants.ELEVATOR_MASS);
        }

        lastKg = ElevatorConstants.MAIN_MOTOR_CONFIGURATION.Slot0.kG;
        double newKg = ElevatorConstants.KG.get() * movingWeight.in(Units.Kilograms);
        if (!Utils.epsilonEquals(lastKg, newKg)) {
            mainMotor
                    .getConfigurator()
                    .apply(ElevatorConstants.MAIN_MOTOR_CONFIGURATION.Slot0.withKG(newKg));
            auxMotor.getConfigurator()
                    .apply(ElevatorConstants.AUX_MOTOR_CONFIGURATION.Slot0.withKG(newKg));
        }
    }
}
