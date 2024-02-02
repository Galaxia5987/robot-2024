package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
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
    private final CommandGenericHID keyBoardControl = new CommandGenericHID(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        GripperConstants.initConstants();
        GripperIO gripperIO;
        ElevatorIO elevatorIO;
        ConveyorIO conveyorIO;
        ConveyorConstants.initialize(Constants.CURRENT_MODE);
        switch (Constants.CURRENT_MODE) {
            case REAL:
                elevatorIO = new ElevatorIOReal();
                conveyorIO = new ConveyorIOSim();
                gripperIO = new GripperIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                conveyorIO = new ConveyorIOSim();
                elevatorIO = new ElevatorIOSim();
                gripperIO = new GripperIOSim();
                break;
        }
        Elevator.initialize(elevatorIO);
        Conveyor.initialize(conveyorIO);
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
        xboxController.x().onTrue(gripper.setWristPosition(Units.Radians.of(0.5).mutableCopy()));
        keyBoardControl
                .button(1)
                .onTrue(conveyor.setVelocity(() -> Units.RotationsPerSecond.of(50).mutableCopy()));
        keyBoardControl
                .button(2)
                .onTrue(conveyor.setVelocity(() -> Units.RotationsPerSecond.of(0).mutableCopy()));
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
