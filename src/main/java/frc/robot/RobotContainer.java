package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.lib.GalacticProxyCommand;
import frc.robot.lib.PoseEstimation;
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
                        () -> -xboxController.getLeftY(),
                        () -> -xboxController.getLeftX(),
                        () -> 0.4 * -xboxController.getRightX(),
                        0.1,
                        () -> true));
    }

    private void configureButtonBindings() {

        //        //        xboxController
        //        //                .rightTrigger()
        //        //                .whileTrue(
        //        //                        Commands.parallel(
        //        //                                        hood.setAngle(
        //        //                                                () ->
        //        //                                                        Units.Degrees.of(
        //        //
        // HoodConstants
        //        //
        //        // .ANGLE_BY_DISTANCE
        //        //
        //        // .getInterpolated(
        //        //
        //        // new InterpolatingDouble(
        //        //
        //        //      PoseEstimation
        //        //
        //        //              .getInstance()
        //        //
        //        //              .getDistanceToSpeaker()))
        //        //
        // .value)
        //        //                                                                .mutableCopy()),
        //        //                                        commandGroups.shootAndConvey(
        //        //                                                () ->
        //        //
        // Units.RotationsPerSecond.of(
        //        //
        // ShooterConstants
        //        //
        //        // .VELOCITY_BY_DISTANCE
        //        //
        //        // .getInterpolated(
        //        //
        //        // new InterpolatingDouble(
        //        //
        //        //      PoseEstimation
        //        //
        //        //              .getInstance()
        //        //
        //        //              .getDistanceToSpeaker()))
        //        //
        // .value)
        //        //                                                                .mutableCopy()))
        //        //                                .until(() -> shooter.atSetpoint() &&
        // hood.atSetpoint())
        //        //                                .andThen(
        //        //                                        gripper.setRollerPower(0.7)
        //        //
        //        // .alongWith(intake.setCenterRollerSpeed(0.5))))
        //        //                .onFalse(
        //        //                        Commands.parallel(
        //        //                                hood.setAngle(() ->
        // Units.Degrees.of(114).mutableCopy()),
        //        //                                conveyor.stop(),
        //        //                                shooter.stop(),
        //        //                                intake.setCenterRollerSpeed(0),
        //        //                                gripper.setRollerPower(0)));
        //        //        xboxController
        //        //                .leftTrigger()
        //        //                .whileTrue(
        //        //                        Commands.parallel(
        //        //                                intake.intake(),
        //        //                                gripper.setRollerPower(0.3)
        //        //                                        .until(gripper::hasNote)
        //        //                                        .andThen(gripper.setRollerPower(0))))
        //        //                .onFalse(Commands.parallel(intake.stop(),
        // gripper.setRollerPower(0)));
        //        xboxController.leftBumper().onTrue(intake.reset(Units.Degrees.of(0)));
        //        xboxController.b().onTrue(Commands.runOnce(swerveDrive::resetGyro));
        //        xboxController
        //                .a()
        //                .whileTrue(elevator.setHeight(Units.Meters.of(0.4).mutableCopy()))
        //                .whileFalse(elevator.setHeight(Units.Meters.of(0).mutableCopy()));
        //        xboxController
        //                .y()
        //                .whileTrue(elevator.setHeight(Units.Meters.of(0.7).mutableCopy()))
        //                .whileFalse(elevator.setHeight(Units.Meters.of(0).mutableCopy()));
        //        xboxController
        //                .x()
        //                .whileTrue(elevator.setHeight(Units.Meters.of(1.0).mutableCopy()))
        //                .whileFalse(elevator.setHeight(Units.Meters.of(0).mutableCopy()));
        //        xboxController
        //                .rightBumper()
        //                .whileTrue(gripper.setWristPosition(Units.Degrees.of(90).mutableCopy()))
        //                .onFalse(gripper.setRollerPower(-0.7));
        //        //        xboxController
        //        //                .rightTrigger()
        //        //                .whileTrue(
        //        //                        Commands.defer(
        //        //                                () ->
        //        //                                        swerveDrive
        //        //                                                .turnCommand(
        //        //                                                        () -> {
        //        //                                                            var toSpeaker =
        //        //
        //        // PoseEstimation.getInstance()
        //        //
        //        // .getPoseRelativeToSpeaker();
        //        //                                                            var res =
        //        //                                                                    new
        // Rotation2d(
        //        //
        //        // toSpeaker.getX(),
        //        //
        //        // toSpeaker.getY());
        //        //                                                            if
        //        // (DriverStation.getAlliance().get()
        //        //                                                                    ==
        //        // DriverStation.Alliance.Red) {
        //        //                                                                res =
        //        //
        // res.minus(
        //        //
        // Rotation2d
        //        //
        //        // .fromDegrees(
        //        //
        //        //      180));
        //        //                                                            }
        //        //                                                            wantedRobotRotation
        // = res;
        //        //                                                            return res;
        //        //                                                        },
        //        //
        //        // Rotation2d.fromDegrees(0.5).getRotations())
        //        //                                                .finallyDo(swerveDrive::lock),
        //        //                                Set.of(swerveDrive)));
        xboxController.a().whileTrue(intake.intake()).whileFalse(intake.stop());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Middle Full Wing");
    }
}
