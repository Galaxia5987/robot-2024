package frc.robot.subsystems.example;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ExampleSubsystem extends SubsystemBase {

    // Single instance in all code of the subsystem
    private static ExampleSubsystem INSTANCE = null;

    // Logger inputs of the subsystem
    private final ExampleSubsystemInputsAutoLogged inputs = ExampleSubsystemIO.inputs;

    // IO of the subsystem
    private final ExampleSubsystemIO io;

    private Mode mode = Mode.POSITION;

    /**
     * Constructor for ExampleSubsystem.
     *
     * @param io IO of the subsystem.
     */
    private ExampleSubsystem(ExampleSubsystemIO io) {
        this.io = io;
    }

    /**
     * Gets the single instance of ExampleSubsystem.
     *
     * @return The single instance of ExampleSubsystem.
     */
    public static ExampleSubsystem getInstance() {
        return INSTANCE;
    }

    public static void initialize(ExampleSubsystemIO io) {
        INSTANCE = new ExampleSubsystem(io);
    }

    /**
     * Sets the position of the subsystem.
     *
     * @param angle The angle of the subsystem to set.
     */
    public void setAngle(Rotation2d angle) {
        inputs.setpointAngle = angle;
        mode = Mode.POSITION;
    }

    public void setPower(double power) {
        inputs.setpointPower = power;
        mode = Mode.POWER;
    }

    /**
     * Get simple command. This command includes only this subsystem.
     *
     * @param angle The angle of the subsystem to set.
     * @return The simple command.
     */
    public Command getSimpleCommand(Rotation2d angle) {
        return new RunCommand(() -> setAngle(angle), this);
    }

    /** Updates the state of the subsystem. */
    @Override
    public void periodic() {
        // Update inputs from IO
        io.updateInputs();
        // Log inputs
        Logger.processInputs("ExampleSubsystem", inputs);

        // Give set point to IO
        if (mode == Mode.POSITION) {
            io.setAngle(inputs.setpointAngle);
        } else {
            io.setPower(inputs.setpointPower);
        }
    }

    public enum Mode {
        POSITION,
        POWER
    }
}
