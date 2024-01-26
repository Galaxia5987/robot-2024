package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.example.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;

    private final Shooter shooter;
    private final CommandGenericHID keyboard = new CommandGenericHID(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ExampleSubsystemIO exampleSubsystemIO;
        ShooterIO shooterIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                exampleSubsystemIO = new ExampleSubsystemIOReal();
                shooterIO = new ShooterIOSim();
                break;
            case SIM:
            case REPLAY:
            default:
                exampleSubsystemIO = new ExampleSubsystemIOSim();
                shooterIO = new ShooterIOSim();
                break;
        }
        ExampleSubsystem.initialize(exampleSubsystemIO);
        Shooter.initialize(shooterIO);

        shooter = Shooter.getInstance();

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

    private void configureButtonBindings() {
        shooter.setDefaultCommand(
                shooter.setVelocity(() -> Units.RotationsPerSecond.of(100).mutableCopy()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
