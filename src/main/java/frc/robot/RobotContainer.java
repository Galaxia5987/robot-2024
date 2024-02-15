package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Elevator elevator;
    private final Gripper gripper;
    private final Hood hood;
    private final Shooter shooter;
    private final SwerveDrive swerveDrive;
    private final CommandXboxController xboxController = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        HoodIO hoodIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                intakeIO = new IntakeIOReal();
//                conveyorIO = new ConveyorIOSim(); // TODO: replace with IOReal
//                elevatorIO = new ElevatorIOReal();
                //                gripperIO = new GripperIOReal();
//                hoodIO = new HoodIOReal();
//                shooterIO = new ShooterIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                intakeIO = new IntakeIOSim();
//                conveyorIO = new ConveyorIOSim();
//                elevatorIO = new ElevatorIOSim();
//                gripperIO = new GripperIOSim();
//                hoodIO = new HoodIOSim();
//                shooterIO = new ShooterIOSim();
                break;
        }
        Intake.initialize(intakeIO);
//        Conveyor.initialize(conveyorIO);
//        Elevator.initialize(elevatorIO);
        //        Gripper.initialize(gripperIO);
//        Hood.initialize(hoodIO);
//        Shooter.initialize(shooterIO);
//        Constants.initSwerve();
//        Constants.initVision();

        swerveDrive = SwerveDrive.getInstance();
        intake = Intake.getInstance();
        conveyor = Conveyor.getInstance();
        elevator = Elevator.getInstance();
        gripper = Gripper.getInstance();
        hood = Hood.getInstance();
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

    private void configureDefaultCommands() {
//        swerveDrive.setDefaultCommand(
//                swerveDrive.driveCommand(
//                        () -> -xboxController.getLeftY(),
//                        () -> -xboxController.getLeftX(),
//                        () -> -xboxController.getRightX(),
//                        0.15,
//                        () -> true));
    }

    private void configureButtonBindings() {
        xboxController.a().whileTrue(intake.setAngle(Units.Degrees.zero().mutableCopy()));
        xboxController.b().whileTrue(intake.setAngle(Units.Degrees.of(100).mutableCopy()));
        xboxController.rightBumper().whileTrue(intake.intake())
                        .onFalse(intake.stop());
        xboxController.x().onTrue(intake.reset(Units.Degrees.zero()));
    }
    private void configureButtonBindings() {
        xboxController.a().whileTrue(hood.setAngle(
                () -> Units.Rotations.of(0.25).mutableCopy()
        ));

        xboxController.leftBumper().onTrue(Commands.runOnce(swerveDrive::resetGyro));
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
