package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.conveyor.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gripper.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.LEDs;
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
//    private final LEDs leds;
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final CommandXboxController driveController = new CommandXboxController(1);
    private final CommandXboxController testController = new CommandXboxController(2);
    private final CommandGroups commandGroups;
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

//        leds = new LEDs(8, 24);
//        leds.setPrimary(Color.kBlue);
//        leds.setSecondary(Color.kYellow);

        Gripper.initialize(gripperIO, () -> Units.Meters.of(0));
        gripper = Gripper.getInstance();
        commandGroups = CommandGroups.getInstance();

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoList", autoChooser);
        NamedCommands.registerCommand("intake", commandGroups.intake());
        NamedCommands.registerCommand("stopIntake", intake.stop());
        NamedCommands.registerCommand("score", commandGroups.feedShooter());
        NamedCommands.registerCommand("finishScore", Commands.runOnce(() -> ShootingManager.getInstance().setShooting(false)));
        NamedCommands.registerCommand("prepareShoot", prepare());
        NamedCommands.registerCommand("shootAndIntake", commandGroups.shootAndIntake());
    }

    private Command prepare() {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                commandGroups.shootAndConvey(
                        ShootingManager.getInstance().getShooterCommandedVelocity()));
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
//        testController.a().onTrue(leds.solid(Color.kGreen, 1, 7));
//        testController.b().onTrue(leds.blink(1, 8, 15));
//        testController.x().onTrue(leds.rainbow(1, 24));
//        testController.y().onTrue(leds.fade(5, 1, 25));
//        testController.leftBumper().onTrue(leds.solid(1, 5));

        driveController.b().onTrue(Commands.runOnce(swerveDrive::resetGyro));
        driveController.y().whileTrue(ShootingManager.getInstance().shootToAmp())
                        .onFalse(Commands.parallel(
                                hood.setAngle(Units.Degrees.of(114).mutableCopy()),
                                shooter.setVelocity(Units.RotationsPerSecond.zero().mutableCopy()),
                                conveyor.stop()));


//        driveController
//                .a()
//                .whileTrue(
//                        Commands.parallel(
//                                        hood.setAngle(Units.Degrees.of(109).mutableCopy()),
//                                        commandGroups.shootAndConvey(
//                                                Units.RotationsPerSecond.of(60).mutableCopy()))
//                                .until(() -> shooter.atSetpoint() && hood.atSetpoint())
//                                .andThen(
//                                        gripper.setRollerPower(0.7)
//                                                .alongWith(intake.setCenterRollerSpeed(0.5))))
//                .onFalse(
//                        Commands.parallel(
//                                hood.setAngle(Units.Degrees.of(114).mutableCopy()),
//                                conveyor.stop(),
//                                shooter.stop(),
//                                intake.setCenterRollerSpeed(0),
//                                gripper.setRollerPower(0)));

        driveController
                .rightTrigger()
                .whileTrue(
                        Commands.parallel(
                                        hood.setAngle(
                                                ShootingManager.getInstance()
                                                        .getHoodCommandedAngle()),
                                        commandGroups.shootAndConvey(
                                                ShootingManager.getInstance()
                                                        .getShooterCommandedVelocity()),
                                        swerveDrive.driveAndAdjust(
                                                ShootingManager.getInstance()
                                                        .getSwerveCommandedAngle(),
                                                () -> -driveController.getLeftY(),
                                                () -> -driveController.getLeftX(),
                                                0.1))
                                .until(ShootingManager.getInstance()::readyToShoot)
                                .andThen(
                                        gripper.setRollerPower(0.7)
                                                .alongWith(intake.setCenterRollerSpeed(0.5))))
                .onFalse(
                        Commands.parallel(
                                hood.setAngle(Units.Degrees.of(114).mutableCopy()),
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
        xboxController
                .x()
                .whileTrue(intake.setAngle(Units.Degrees.of(-130).mutableCopy()))
                .onFalse(intake.reset(Units.Degrees.of(0)));

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

        xboxController.a().onTrue(elevator.unlock());
        xboxController.y().onTrue(elevator.lock());

//        xboxController.a().whileTrue(elevator.setHeight(Units.Meters.of(0).mutableCopy()));
//        xboxController.x().whileTrue(elevator.setHeight(Units.Meters.of(0.2).mutableCopy()));
//        xboxController.y().whileTrue(elevator.setHeight(Units.Meters.of(0.48).mutableCopy()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("AH123");
    }
}
