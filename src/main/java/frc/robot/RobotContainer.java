package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.scoreStates.*;
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
import java.util.Optional;

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
    private final CommandXboxController driveController = new CommandXboxController(1);
    private final CommandGroups commandGroups;
    private final SendableChooser<Command> autoChooser;
    private final ScoreState ampState;
    private final ScoreState shootState;
    private final ScoreState climbState;
    private final StateManager stateManager;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        HoodIO hoodIO;
        IntakeIO intakeIO;
        ConveyorIO conveyorIO;
        ShooterIO shooterIO;
        GripperIO gripperIO;
        ElevatorIO elevatorIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                intakeIO = new IntakeIOReal();
                conveyorIO = new ConveyorIOReal();
                gripperIO = new GripperIOReal();
                hoodIO = new HoodIOReal();
                shooterIO = new ShooterIOReal();
                elevatorIO = new ElevatorIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                intakeIO = new IntakeIOSim();
                conveyorIO = new ConveyorIOSim();
                gripperIO = new GripperIOSim();
                hoodIO = new HoodIOSim();
                shooterIO = new ShooterIOSim();
                elevatorIO = new ElevatorIOSim();
                break;
        }
        Intake.initialize(intakeIO);
        Conveyor.initialize(conveyorIO);
        Elevator.initialize(elevatorIO);
        Gripper.initialize(gripperIO, () -> Units.Meters.of(0));
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

        Gripper.initialize(gripperIO, () -> Units.Meters.of(0));
        gripper = Gripper.getInstance();
        commandGroups = CommandGroups.getInstance();

        ampState = new AmpState();
        shootState = new ShootState();
        climbState = new ClimbState();
        stateManager = StateManager.getInstance(shootState);

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
        NamedCommands.registerCommand("intake", commandGroups.intake());
        NamedCommands.registerCommand("stopIntake", intake.stop());
        NamedCommands.registerCommand(
                "score", stateManager.getCurrentState().score(Optional.empty(), true)); //
        //         scoreShooter
        NamedCommands.registerCommand("finishScore", gripper.stopGripper());
        NamedCommands.registerCommand(
                "prepareShoot", stateManager.getCurrentState().prepareSubsystems());
        autoChooser = AutoBuilder.buildAutoChooser("Safety B");
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
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        () -> 0.6 * -driveController.getRightX(), // 0.6
                        0.1,
                        () -> true));

        elevator.setDefaultCommand(
                elevator.manualElevator(
                        () ->
                                MathUtil.applyDeadband(
                                        -xboxController.getLeftTriggerAxis()
                                                + xboxController.getRightTriggerAxis(),
                                        0.15)));

        gripper.setDefaultCommand(
                gripper.setWristPower(
                        () -> MathUtil.applyDeadband(-xboxController.getRightY(), 0.15) * 0.6));
    }

    private void configureButtonBindings() {
        driveController.b().onTrue(Commands.runOnce(swerveDrive::resetGyro));
        driveController
                .y()
                .whileTrue(
                        stateManager
                                .setCurrentState(ampState)
                                .andThen(
                                        stateManager
                                                .getCurrentState()
                                                .score(Optional.empty(), false)))
                .onFalse(Commands.parallel(commandGroups.stopHoodShooterConveyorGripper()));

        driveController
                .rightTrigger()
                .whileTrue(
                        stateManager.getCurrentState().score(Optional.of(driveController), false))
                .onFalse(commandGroups.stopAllSubsystems());

        driveController
                .leftTrigger()
                .whileTrue(commandGroups.intake())
                .onFalse(Commands.parallel(intake.stop(), gripper.setRollerPower(0)));

        driveController
                .rightBumper()
                .whileTrue(intake.outtake().alongWith(gripper.setRollerPower(-0.7)))
                .onFalse(intake.stop().alongWith(gripper.setRollerPower(0)));

        xboxController
                .x()
                .whileTrue(intake.setAngle(Units.Degrees.of(-130).mutableCopy()))
                .onFalse(intake.reset(Units.Degrees.of(0)));

        xboxController.start().onTrue(elevator.lock());
        xboxController.back().onTrue(elevator.unlock());
        xboxController
                .rightBumper()
                .whileTrue(gripper.setRollerPower(0.4))
                .onFalse(gripper.setRollerPower(0));

        xboxController
                .leftBumper()
                .whileTrue(gripper.setRollerPower(-0.4))
                .onFalse(gripper.setRollerPower(0));
        // TODO: make force shoot
        xboxController
                .x()
                .onTrue(intake.setAngle(Units.Degrees.of(-140).mutableCopy()))
                .onFalse(intake.reset(Units.Degrees.zero().mutableCopy()));

        xboxController.a().onTrue(elevator.unlock());
        xboxController.y().onTrue(elevator.lock());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
