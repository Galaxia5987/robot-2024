package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.gripper.*;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;
    private final Elevator elevator;
    private final Gripper gripper;
    private final Conveyor conveyor;
    private final SwerveDrive swerveDrive;
    private final CommandXboxController xboxController = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        GripperConstants.initConstants();
        GripperIO gripperIO;
        ElevatorIO elevatorIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                elevatorIO = new ElevatorIOReal();
                gripperIO = new GripperIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                elevatorIO = new ElevatorIOSim();
                elevatorIO = new ElevatorIOSim();
                gripperIO = new GripperIOSim();
                break;
        }
        Elevator.initialize(elevatorIO);
        Gripper.initialize(gripperIO);
        Constants.initSwerve();
        Constants.initVision();

        gripper = Gripper.getInstance();
        swerveDrive = SwerveDrive.getInstance();
        elevator = Elevator.getInstance();
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
