package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.lib.GalacticProxyCommand;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
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
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
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
    private final CommandXboxController driveController = new CommandXboxController(1);
    private final CommandPS5Controller ps5Controller = new CommandPS5Controller(1);
    private final ShootState shootState;
    private final AmpState ampState;
    private final ClimbState climbState;
    private final PoseEstimation poseEstimation;
    private CommandGroups commandGroups;
    private ScoreState currentState;
    private final SendableChooser<Command> autoChooser;

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

        currentState = new ShootState();
        shootState = new ShootState();
        ampState = new AmpState();
        climbState = new ClimbState();

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
        poseEstimation = PoseEstimation.getInstance();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoList", autoChooser);
        NamedCommands.registerCommand("intake", commandGroups.intake());
        NamedCommands.registerCommand("stopIntake", intake.stop());
        NamedCommands.registerCommand("score", commandGroups.feedShooter());
        NamedCommands.registerCommand("prepareShoot", updateScoreState());
    }

    private Command prepare() {
        return Commands.parallel(
                hood.setAngle(
                        () ->
                                Units.Degrees.of(
                                                HoodConstants.ANGLE_BY_DISTANCE.getInterpolated(
                                                                new InterpolatingDouble(
                                                                        PoseEstimation.getInstance()
                                                                                .getDistanceToSpeaker()))
                                                        .value)
                                        .mutableCopy()),
                commandGroups.shootAndConvey(
                        () ->
                                Units.RotationsPerSecond.of(
                                                ShooterConstants.VELOCITY_BY_DISTANCE
                                                        .getInterpolated(
                                                                new InterpolatingDouble(
                                                                        PoseEstimation.getInstance()
                                                                                .getDistanceToSpeaker()))
                                                        .value)
                                        .mutableCopy()));
    }

    private Command updateScoreState() {
        return new GalacticProxyCommand(
                () ->
                        currentState
                                .calculateTargets()
                                .andThen(currentState.prepareSubsystems())
                                .repeatedly());
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
                        () -> 0.6 * -driveController.getRightX(),
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
                        () -> MathUtil.applyDeadband(-xboxController.getLeftY(), 0.15)));
    }

    private void configureButtonBindings() {
        driveController.b().onTrue(Commands.runOnce(swerveDrive::resetGyro));

        driveController
                .a()
                .whileTrue(
                        Commands.parallel(
                                        hood.setAngle(() -> Units.Degrees.of(109).mutableCopy()),
                                        commandGroups.shootAndConvey(
                                                () ->
                                                        Units.RotationsPerSecond.of(60)
                                                                .mutableCopy()))
                                .until(() -> shooter.atSetpoint() && hood.atSetpoint())
                                .andThen(
                                        gripper.setRollerPower(0.7)
                                                .alongWith(intake.setCenterRollerSpeed(0.5))))
                .onFalse(
                        Commands.parallel(
                                hood.setAngle(() -> Units.Degrees.of(114).mutableCopy()),
                                conveyor.stop(),
                                shooter.stop(),
                                intake.setCenterRollerSpeed(0),
                                gripper.setRollerPower(0)));

        driveController
                .rightTrigger()
                .whileTrue(
                        Commands.parallel(
                                        hood.setAngle(
                                                () ->
                                                        Units.Degrees.of(
                                                                        HoodConstants
                                                                                .ANGLE_BY_DISTANCE
                                                                                .getInterpolated(
                                                                                        new InterpolatingDouble(
                                                                                                PoseEstimation
                                                                                                        .getInstance()
                                                                                                        .getDistanceToSpeaker()))
                                                                                .value)
                                                                .mutableCopy()),
                                        commandGroups.shootAndConvey(
                                                () ->
                                                        Units.RotationsPerSecond.of(
                                                                        ShooterConstants
                                                                                .VELOCITY_BY_DISTANCE
                                                                                .getInterpolated(
                                                                                        new InterpolatingDouble(
                                                                                                PoseEstimation
                                                                                                        .getInstance()
                                                                                                        .getDistanceToSpeaker()))
                                                                                .value)
                                                                .mutableCopy()))
                                .until(() -> shooter.atSetpoint() && hood.atSetpoint())
                                .andThen(
                                        gripper.setRollerPower(0.7)
                                                .alongWith(intake.setCenterRollerSpeed(0.5))))
                .onFalse(
                        Commands.parallel(
                                hood.setAngle(() -> Units.Degrees.of(114).mutableCopy()),
                                conveyor.stop(),
                                shooter.stop(),
                                intake.setCenterRollerSpeed(0),
                                gripper.setRollerPower(0)));

        driveController
                .leftTrigger()
                .whileTrue(
                        Commands.parallel(
                                intake.intake(),
                                gripper.setRollerPower(0.3)
                                        .until(gripper::hasNote)
                                        .andThen(gripper.setRollerPower(0))))
                .onFalse(Commands.parallel(intake.stop(), gripper.setRollerPower(0)));
        driveController.rightBumper().whileTrue(intake.outtake()).onFalse(intake.stop());
        driveController
                .x()
                .whileTrue(intake.setAngle(Units.Degrees.of(-130).mutableCopy()))
                .onFalse(intake.reset(Units.Degrees.of(0)));

        //        xboxController.b().whileTrue(hood.setAngle(()->
        // Units.Degrees.of(65).mutableCopy()));

        xboxController
                .rightBumper()
                .whileTrue(gripper.setRollerPower(0.4))
                .onFalse(gripper.setRollerPower(0));
        xboxController
                .leftBumper()
                .whileTrue(gripper.setRollerPower(-0.4))
                .onFalse(gripper.setRollerPower(0));

        xboxController
                .b()
                .whileTrue(intake.setAngle(IntakeConstants.IntakePose.DOWN))
                .onFalse(intake.setAngle(IntakeConstants.IntakePose.UP));

        xboxController.a().whileTrue(elevator.setHeight(Units.Meters.of(0).mutableCopy()));
        xboxController.x().whileTrue(elevator.setHeight(Units.Meters.of(0.2).mutableCopy()));
        xboxController.y().whileTrue(elevator.setHeight(Units.Meters.of(0.48).mutableCopy()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //        return new InstantCommand(
        //                () ->
        //                        swerveDrive.resetPose(
        //                                PathPlannerAuto.getStaringPoseFromAutoFile(
        //                                        "New Auto")),
        //                                swerveDrive)
        //                        .andThen(
        return new PathPlannerAuto("D345");
    }
}
