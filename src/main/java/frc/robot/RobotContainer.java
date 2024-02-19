package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.lib.GalacticProxyCommand;
import frc.robot.scoreStates.AmpState;
import frc.robot.scoreStates.ClimbState;
import frc.robot.scoreStates.ScoreState;
import frc.robot.scoreStates.ShootState;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOReal;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOReal;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.intake.*;
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
    private final ShootState shootState;
    private final AmpState ampState;
    private final ClimbState climbState;
    private final SendableChooser<Command> autoChooser;
    private CommandGroups commandGroups;
    private ScoreState currentState;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        IntakeIO intakeIO;
        ConveyorIO conveyorIO;
        ElevatorIO elevatorIO;
        GripperIO gripperIO;
        HoodIO hoodIO;
        ShooterIO shooterIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                intakeIO = new IntakeIOReal();
                conveyorIO = new ConveyorIOReal();
                elevatorIO = new ElevatorIOReal();
                gripperIO = new GripperIOReal();
                hoodIO = new HoodIOReal();
                shooterIO = new ShooterIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                intakeIO = new IntakeIOSim();
                conveyorIO = new ConveyorIOSim();
                elevatorIO = new ElevatorIOSim();
                gripperIO = new GripperIOSim();
                hoodIO = new HoodIOSim();
                shooterIO = new ShooterIOSim();
                break;
        }
        Intake.initialize(intakeIO);
        Conveyor.initialize(conveyorIO);
        Elevator.initialize(elevatorIO);
        Hood.initialize(hoodIO);
        Shooter.initialize(shooterIO);
        Constants.initSwerve();
        Constants.initVision();

        swerveDrive = SwerveDrive.getInstance();
        intake = Intake.getInstance();
        conveyor = Conveyor.getInstance();
        elevator = Elevator.getInstance();
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();

        Gripper.initialize(gripperIO, elevator::getCarriageHeight);
        gripper = Gripper.getInstance();
        commandGroups = CommandGroups.getInstance();

        currentState = new ShootState();
        shootState = new ShootState();
        ampState = new AmpState();
        climbState = new ClimbState();

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();

        NamedCommands.registerCommand("intake", commandGroups.intake());
        NamedCommands.registerCommand("prepareShoot", updateScoreState());
        NamedCommands.registerCommand("score", currentState.score());
        //        NamedCommands.registerCommand("resetPose", new
        // InstantCommand(()->swerveDrive.resetPose(PathPlannerAuto.getStaringPoseFromAutoFile("LowerFullWing"))));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoList", autoChooser);
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
        intake.setDefaultCommand(intake.setAngle(IntakeConstants.IntakePose.UP));
    }

    private Command updateScoreState() {
        return new GalacticProxyCommand(
                () ->
                        currentState
                                .calculateTargets()
                                .andThen(currentState.prepareSubsystems())
                                .repeatedly());
    }

    private void configureButtonBindings() {
        CommandScheduler.getInstance().onCommandInitialize(System.out::println);
        xboxController.y().whileTrue(commandGroups.intake()).onFalse(intake.stop());
        xboxController
                .a()
                .onTrue(
                        Commands.runOnce(() -> currentState = shootState)
                                .andThen(updateScoreState()));
        xboxController
                .b()
                .onTrue(
                        Commands.runOnce(() -> currentState = ampState)
                                .andThen(updateScoreState()));
        xboxController
                .x()
                .onTrue(
                        Commands.runOnce(() -> currentState = climbState)
                                .andThen(updateScoreState()));

        xboxController
                .rightTrigger(0.1)
                .onTrue(new GalacticProxyCommand(() -> currentState.score()))
                .onFalse(new GalacticProxyCommand(() -> currentState.finalizeScore()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return new InstantCommand(
                        () ->
                                swerveDrive.resetPose(
                                        PathPlannerAuto.getStaringPoseFromAutoFile(
                                                "Middle Full Wing")),
                        swerveDrive)
                .andThen(
                        AutoBuilder.buildAuto(
                                ("Middle Full Wing"))); // new PathPlannerAuto("LowerFullWing");
    }
}
