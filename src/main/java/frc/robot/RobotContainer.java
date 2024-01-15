package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.example.*;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ExampleSubsystemIO exampleSubsystemIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                exampleSubsystemIO = new ExampleSubsystemIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                exampleSubsystemIO = new ExampleSubsystemIOSim();
                break;
        }
        ExampleSubsystem.initialize(exampleSubsystemIO);

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {}

    private void configureButtonBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
