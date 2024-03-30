package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperIO io;
    private final GripperInputsAutoLogged inputs = GripperIO.inputs;
    private final Timer timer = new Timer();

    private Gripper(GripperIO io) {
        this.io = io;

        timer.start();
        timer.reset();
    }

    public static Gripper getInstance() {
        return INSTANCE;
    }

    public static void initialize(GripperIO io) {
        INSTANCE = new Gripper(io);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public Command setRollerPower(double power) {
        return run(() -> io.setRollerMotorPower(power)).withName("set roller power");
    }

    @Override
    public void periodic() {
        io.updateInputs();
        if (timer.advanceIfElapsed(0.0)) {
            Logger.processInputs(this.getClass().getSimpleName(), inputs);
        }
    }
}
