package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Elevator INSTANCE;
    private final ElevatorInputs inputs = new ElevatorInputs();
    private final ElevatorIO io;
    private final Mechanism2d mechanism2d = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);
    private final MechanismRoot2d root = mechanism2d.getRoot("Elevator", ROOT_X, ROOT_Y);
    private ControlModeValue controlMode;
    private Command lastCommand = null;
    private Command currentCommand = null;
    private boolean changedToDefaultCommand = false;

    private Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void setPower(double power) {
        inputs.power = power;
        inputs.controlMode = ElevatorIO.ControlMode.PERCENT_OUTPUT;
    }
    

    public void setHeight(MutableMeasure<Distance> height) {
        inputs.height = height;
        inputs.controlMode = ElevatorIO.ControlMode.POSITION;
    }

    public MutableMeasure<Distance> getHeight() {
        return inputs.height;
    }

    public MutableMeasure<Distance> getHeightSetpoint() {
        return inputs.heightSetpoint;
    }

    public void stop() {
        inputs.controlMode = null;
    }
}
