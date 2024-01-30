package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import frc.robot.subsystems.example.ExampleSubsystemIOReal;
import frc.robot.subsystems.example.ExampleSubsystemIOSim;
import frc.robot.swerve.*;

import java.util.function.Supplier;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive;
    private final Conveyor conveyor;
    private final CommandXboxController xboxController = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ConveyorConstants.initialize(Constants.CURRENT_MODE
        );
        ExampleSubsystemIO exampleSubsystemIO;
        ConveyorIO conveyorIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                exampleSubsystemIO = new ExampleSubsystemIOReal();
                conveyorIO  = new ConveyorIOSim();
                break;
            case SIM:
            case REPLAY:
            default:
                exampleSubsystemIO = new ExampleSubsystemIOSim();
                conveyorIO = new ConveyorIOSim();
                break;
        }
        ExampleSubsystem.initialize(exampleSubsystemIO);
        Conveyor.initialize(conveyorIO);
        Constants.initSwerve();
        Constants.initVision();


        swerveDrive = SwerveDrive.getInstance();
        conveyor = Conveyor.getInstance();

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

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(
                swerveDrive.driveCommand(
                        () -> -xboxController.getLeftY(),
                        () -> -xboxController.getLeftX(),
                        () -> -xboxController.getRightX(),
                        0.15,
                        () -> true));
    }

    private void configureButtonBindings() {
        xboxController.a().onTrue(conveyor.setVelocity(()-> Units.RotationsPerSecond.of(10).mutableCopy()));
        xboxController.b().onTrue(conveyor.setVelocity(()-> Units.RotationsPerSecond.of(20).mutableCopy()));
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
