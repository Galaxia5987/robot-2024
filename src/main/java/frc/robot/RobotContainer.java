package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.scoreStates.ScoreState;
import frc.robot.subsystems.ShootingManager;
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
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Elevator elevator;
    private final Gripper gripper;
    private final Hood hood;
    private final Shooter shooter;
    private final SwerveDrive swerveDrive;
    private final LEDs leds;
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final CommandPS5Controller driveController = new CommandPS5Controller(1);
    private final CommandXboxController testController = new CommandXboxController(2);
    private final CommandGroups commandGroups;
    private final SendableChooser<Command> autoChooser;
    @Getter @Setter private boolean isForceShooting = false;

    @AutoLogOutput private ScoreState.State state = ScoreState.State.SHOOT;
    public boolean isIntaking = false;

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

        leds = new LEDs(8, 120);
        leds.setPrimary(Color.kDeepPink);
        leds.setSecondary(Color.kYellow);

        Gripper.initialize(gripperIO, () -> Units.Meters.of(0));
        gripper = Gripper.getInstance();
        commandGroups = CommandGroups.getInstance();

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
        NamedCommands.registerCommand("intake", commandGroups.intake());
        NamedCommands.registerCommand(
                "print1", Commands.print("CapsLockIsBetter!!!").repeatedly().withTimeout(2));
        NamedCommands.registerCommand(
                "print2", Commands.print("GaiaWasRightInPrintOne!!!").repeatedly().withTimeout(2));
        NamedCommands.registerCommand("stopIntake", intake.stop(false).withTimeout(0.05));
        NamedCommands.registerCommand("retractIntake", intake.stop());
        NamedCommands.registerCommand("score", commandGroups.feedShooter(() -> isForceShooting));
        NamedCommands.registerCommand("finishScore", gripper.setRollerPower(0).withTimeout(0.05));
        NamedCommands.registerCommand(
                "followPathRotation",
                        Commands.runOnce(() -> ShootingManager.getInstance().setShooting(false)));
        NamedCommands.registerCommand("followPathSpeeds", Commands.runOnce(()->setIntaking(false)));
        NamedCommands.registerCommand("useNoteDetection", Commands.runOnce(()->setIntaking(true)));

        NamedCommands.registerCommand("prepareShoot", prepare());
        NamedCommands.registerCommand("shootAndIntake", commandGroups.shootAndIntake());
        NamedCommands.registerCommand(
                "adjustToTarget",
                Commands.runOnce(() -> ShootingManager.getInstance().setShooting(true)));
        NamedCommands.registerCommand(
                "setVisionMeasurementAuto",
                Commands.runOnce(
                        () ->
                                Constants.VISION_MEASUREMENT_MULTIPLIER =
                                        Constants.AUTO_VISION_MEASUREMENT_MULTIPLIER));

        PPHolonomicDriveController.setRotationTargetOverride(
                () -> {
                    if (ShootingManager.getInstance().isShooting()) {
                        return Optional.of(
                                new Rotation2d(
                                        ShootingManager.getInstance().getSwerveCommandedAngle()));
                    }
                    return Optional.empty();
                });
        autoChooser = AutoBuilder.buildAutoChooser("Safety B");
        SmartDashboard.putData("autoList", autoChooser);
    }

    private void setIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private Command prepare() {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                commandGroups.shootAndConvey(
                        ShootingManager.getInstance().getShooterCommandedVelocity()));
    }

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(
                swerveDrive.driveCommand(
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        () -> 0.4 * -driveController.getRightX(), // 0.6
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
        testController.a().onTrue(leds.solid(new Color("#009cbd"), 1, 120));
        testController.b().whileTrue(leds.blink(2, 1, 120));
        testController.x().whileTrue(leds.rainbow(1, 120));
        testController.y().whileTrue(leds.fade(6, 1, 120));
        testController.rightBumper().onTrue(commandGroups.allBits());

        driveController.triangle().onTrue(Commands.runOnce(swerveDrive::resetGyro));
        driveController
                .circle()
                .whileTrue(
                        commandGroups
                                .shootAndConvey(Units.RotationsPerSecond.of(50).mutableCopy())
                                .alongWith(
                                        hood.setAngle(Units.Degrees.of(107).mutableCopy()),
                                        commandGroups.feedWithWait(
                                                () ->
                                                        (shooter.atSetpoint()
                                                                        && hood.getAngle()
                                                                                        .minus(
                                                                                                Units
                                                                                                        .Degrees
                                                                                                        .of(
                                                                                                                107))
                                                                                        .in(
                                                                                                Units
                                                                                                        .Degrees)
                                                                                < 2)
                                                                || isForceShooting)))
                .onFalse(commandGroups.stopShooting());
        driveController
                .square()
                .whileTrue(
                        commandGroups
                                .shootAndConvey(Units.RotationsPerSecond.of(50).mutableCopy())
                                .alongWith(
                                        hood.setAngle(Units.Degrees.of(108).mutableCopy()),
                                        commandGroups.feedWithWait(
                                                () ->
                                                        (shooter.atSetpoint()
                                                                        && hood.getAngle()
                                                                                        .minus(
                                                                                                Units
                                                                                                        .Degrees
                                                                                                        .of(
                                                                                                                107))
                                                                                        .in(
                                                                                                Units
                                                                                                        .Degrees)
                                                                                < 2)
                                                                || isForceShooting)))
                .onFalse(commandGroups.stopShooting());

        driveController
                .R2()
                .whileTrue(
                        Commands.either(
                                commandGroups
                                        .shootToSpeaker(driveController)
                                        .alongWith(
                                                commandGroups.feedShooter(this::isForceShooting)),
                                commandGroups.shootToAmp(),
                                () -> state == ScoreState.State.SHOOT))
                .onFalse(commandGroups.stopShooting());

        driveController
                .L2()
                .whileTrue(commandGroups.intake())
                .onFalse(Commands.parallel(intake.stop(), gripper.setRollerPower(0)));

        driveController
                .R1()
                .whileTrue(intake.outtake().alongWith(gripper.setRollerPower(-0.7)))
                .onFalse(intake.stop().alongWith(gripper.setRollerPower(0)));

        xboxController.start().onTrue(elevator.lock());
        xboxController.back().onTrue(elevator.unlock());
        xboxController
                .leftBumper()
                .whileTrue(commandGroups.closeSpeakerWarmup())
                .onFalse(commandGroups.stopShooting());
        xboxController
                .rightBumper()
                .whileTrue(gripper.setRollerPower(0.4))
                .onFalse(gripper.setRollerPower(0));

        xboxController.b().onTrue(Commands.runOnce(() -> state = ScoreState.State.SHOOT));
        xboxController.a().onTrue(Commands.runOnce(() -> state = ScoreState.State.AMP));
        xboxController.y().onTrue(commandGroups.shootToSpeaker());
        xboxController
                .x()
                .onTrue(intake.setAnglePower(-0.3))
                .onFalse(intake.reset(Units.Degrees.zero().mutableCopy()));

        xboxController
                .leftStick()
                .whileTrue(Commands.runOnce(() -> ShootingManager.getInstance().setLockShoot(true)))
                .onFalse(Commands.runOnce(() -> ShootingManager.getInstance().setLockShoot(false)));
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
