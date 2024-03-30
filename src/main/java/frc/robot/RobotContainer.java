package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.lib.GalacticProxyCommand;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOReal;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.gripper.*;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsDefaultCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Climb climb;
    private final Gripper gripper;
    private final Hood hood;
    private final Shooter shooter;
    private final SwerveDrive swerveDrive;
    public static final LEDs leds = new LEDs(9, 58);
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandPS5Controller operatorController = new CommandPS5Controller(1);
    private final CommandXboxController testController = new CommandXboxController(2);
    private final CommandGroups commandGroups;
    private final SendableChooser<String> autoChooser;
    private final HashMap<String, PathPlannerAuto> autos = new HashMap<>();
    @Getter @Setter private boolean isForceShooting = false;
    @Getter private Constants.State state = Constants.State.SHOOT;

    @AutoLogOutput boolean useNoteDetection = false;

    public final MutableMeasure<Angle> hoodTuningAngle = Units.Degrees.of(110).mutableCopy();
    public final MutableMeasure<Velocity<Angle>> shooterTuningVelocity =
            Units.RotationsPerSecond.of(50).mutableCopy();

    public int pathIndex = 0;
    private PathPlannerAuto pathPlannerAuto;
    private List<PathPoint> pathPoints =
            new ArrayList<>() {
                {
                    add(new PathPoint(new Translation2d()));
                }
            };
    private final Timer scoreTimer = new Timer();

    private boolean ampPressed = true;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        HoodIO hoodIO;
        IntakeIO intakeIO;
        ConveyorIO conveyorIO;
        ShooterIO shooterIO;
        GripperIO gripperIO;
        ClimbIO climbIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                intakeIO = new IntakeIOReal();
                conveyorIO = new ConveyorIOReal();
                gripperIO = new GripperIOReal();
                hoodIO = new HoodIOReal();
                shooterIO = new ShooterIOReal();
                climbIO = new ClimbIOReal();
                break;
            case SIM:
                intakeIO = new IntakeIOSim();
                conveyorIO = new ConveyorIOSim();
                gripperIO = new GripperIOSim();
                hoodIO = new HoodIOSim();
                shooterIO = new ShooterIOSim();
                climbIO = new ClimbIOSim();
                break;
            case REPLAY:
            default:
                intakeIO = new IntakeIO() {};
                conveyorIO = new ConveyorIO() {};
                gripperIO = new GripperIO() {};
                hoodIO = new HoodIO() {};
                shooterIO = new ShooterIO() {};
                climbIO = new ClimbIO() {};
                break;
        }
        Intake.initialize(intakeIO);
        Conveyor.initialize(conveyorIO);
        Climb.initialize(climbIO);
        Gripper.initialize(gripperIO);
        Hood.initialize(hoodIO);
        Shooter.initialize(shooterIO);
        Constants.initSwerve();
        Constants.initVision();

        swerveDrive = SwerveDrive.getInstance();
        intake = Intake.getInstance();
        conveyor = Conveyor.getInstance();
        climb = Climb.getInstance();
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();

        gripper = Gripper.getInstance();
        commandGroups = CommandGroups.getInstance();

        scoreTimer.start();
        scoreTimer.reset();

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
        NamedCommands.registerCommand("intake", commandGroups.intake(Commands.none()));
        NamedCommands.registerCommand(
                "print1", Commands.print("CapsLockIsBetter!!!").repeatedly().withTimeout(2));
        NamedCommands.registerCommand(
                "print2", Commands.print("GaiaWasRightInPrintOne!!!").repeatedly().withTimeout(2));
        NamedCommands.registerCommand("stopIntake", intake.stop(false).withTimeout(0.05));
        NamedCommands.registerCommand("retractIntake", intake.stop());
        NamedCommands.registerCommand(
                "score",
                Commands.runOnce(scoreTimer::reset)
                        .andThen(
                                commandGroups
                                        .feedShooter(() -> scoreTimer.hasElapsed(3))
                                        .andThen(
                                                () ->
                                                        pathIndex =
                                                                Math.min(
                                                                        pathIndex + 1,
                                                                        pathPoints.size() - 1))));
        NamedCommands.registerCommand("finishScore", gripper.setRollerPower(0).withTimeout(0.05));
        NamedCommands.registerCommand(
                "followPathRotation",
                Commands.runOnce(() -> ShootingManager.getInstance().setShooting(false)));
        NamedCommands.registerCommand(
                "followPathSpeeds", Commands.runOnce(() -> setUseNoteDetection(false)));
        NamedCommands.registerCommand(
                "useNoteDetection", Commands.runOnce(() -> setUseNoteDetection(true)));

        NamedCommands.registerCommand("prepareShoot", prepareShooter());
        NamedCommands.registerCommand("closeShoot", closeShoot());
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
        NamedCommands.registerCommand("shoot", prepareShooter());
        NamedCommands.registerCommand(
                "stopShooter",
                Commands.parallel(
                        hood.setAngle(Units.Degrees.of(114).mutableCopy()),
                        shooter.stop(),
                        conveyor.stop()));

        PPHolonomicDriveController.setRotationTargetOverride(
                () -> {
                    if (ShootingManager.getInstance().isShooting()) {
                        return Optional.of(
                                new Rotation2d(
                                        ShootingManager.getInstance().getSwerveCommandedAngle()));
                    }
                    return Optional.empty();
                });
        autoChooser =
                new SendableChooser<>() {
                    {
                        List<String> paths = AutoBuilder.getAllAutoNames();
                        paths.forEach(
                                (path) -> {
                                    addOption(path, path);
                                    System.out.println("Creating - " + path);
                                    autos.put(path, new PathPlannerAuto(path));
                                });
                    }
                };
        autoChooser.setDefaultOption("Safety score", "Safety score");
        SmartDashboard.putData("autoList", autoChooser);
    }

    private void setUseNoteDetection(boolean useNoteDetection) {
        this.useNoteDetection = useNoteDetection;
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    public Command prepareShooter() {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                commandGroups.shootAndConvey(
                        ShootingManager.getInstance().getShooterCommandedVelocity(), true));
    }

    public Translation2d getAutoToSpeaker() {
        if (pathPoints.isEmpty()) {
            return new Translation2d();
        }
        return VisionConstants.getSpeakerPose()
                .minus(
                        Constants.isRed()
                                ? GeometryUtil.flipFieldPosition(pathPoints.get(pathIndex).position)
                                : pathPoints.get(pathIndex).position);
    }

    public Command closeShoot() {
        return commandGroups
                .shootAndConvey(shooterTuningVelocity, false)
                .raceWith(
                        hood.setAngle(hoodTuningAngle),
                        commandGroups.feedWithWait(
                                () ->
                                        (shooter.atSetpoint()
                                                        && conveyor.atSetpoint()
                                                        && hood.atSetpoint())
                                                || isForceShooting));
    }

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(
                swerveDrive.driveCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> 0.6 * -driverController.getRightX(), // 0.6
                        0.1,
                        () -> true));

        climb.setDefaultCommand(
                climb.setPower(
                        () ->
                                MathUtil.applyDeadband(
                                        -(operatorController.getL2Axis() + 1) / 2
                                                + (operatorController.getR2Axis() + 1) / 2,
                                        0.15)));

        leds.setDefaultCommand(
                new LEDsDefaultCommand(leds, driverController.leftTrigger()).ignoringDisable(true));
    }

    private void configureButtonBindings() {
        testController.rightBumper().onTrue(commandGroups.allBits(operatorController));
        testController.leftBumper().onTrue(commandGroups.swerveBit());
        testController.a().onTrue(commandGroups.openClimb());

        driverController
                .a()
                .whileTrue(commandGroups.superPoop(operatorController, () -> isForceShooting))
                .onFalse(commandGroups.stopShooting());
        driverController.y().onTrue(Commands.runOnce(swerveDrive::resetGyro));
        driverController.b().whileTrue(closeShoot()).onFalse(commandGroups.stopShooting());
        driverController
                .x()
                .whileTrue(
                        commandGroups
                                .shootAndConvey(
                                        Units.RotationsPerSecond.of(73).mutableCopy(), false)
                                .alongWith(
                                        hood.setAngle(Units.Degrees.of(84).mutableCopy()),
                                        commandGroups.feedWithWait(
                                                () ->
                                                        (shooter.atSetpoint() && hood.atSetpoint()
                                                                || isForceShooting))))
                .onFalse(commandGroups.stopShooting());

        driverController
                .rightTrigger()
                .whileTrue(
                        commandGroups
                                .shootToSpeaker(driverController)
                                .alongWith(commandGroups.feedShooter(this::isForceShooting)))
                .onFalse(commandGroups.stopShooting());
        driverController
                .rightTrigger()
                .onTrue(Commands.runOnce(() -> state = Constants.State.SHOOT));

        driverController
                .leftTrigger()
                .whileTrue(commandGroups.intake(Commands.none()))
                .onFalse(Commands.parallel(intake.stop(), gripper.setRollerPower(0)));

        driverController
                .rightBumper()
                .whileTrue(intake.outtake().alongWith(gripper.setRollerPower(-0.7)))
                .onFalse(intake.stop().alongWith(gripper.setRollerPower(0)));

        driverController.leftBumper().whileTrue(commandGroups.adjustToAmp(driverController));
        driverController
                .leftBumper()
                .whileTrue(commandGroups.shootToAmp(operatorController))
                .onFalse(commandGroups.stopShooting());
        driverController
                .leftBumper()
                .onTrue(Commands.runOnce(() -> state = Constants.State.AMP))
                .onFalse(Commands.runOnce(() -> state = Constants.State.SHOOT));

        operatorController.options().onTrue(climb.lock());
        operatorController
                .create()
                .onTrue(
                        climb.unlock()
                                .alongWith(Commands.runOnce(() -> state = Constants.State.CLIMB)));
        operatorController
                .R1()
                .whileTrue(gripper.setRollerPower(0.4))
                .onFalse(gripper.setRollerPower(0));
        operatorController
                .L1()
                .whileTrue(gripper.setRollerPower(-0.4))
                .onFalse(gripper.setRollerPower(0));

        operatorController.triangle().onTrue(commandGroups.shootToSpeaker());
        operatorController
                .square()
                .onTrue(intake.setAnglePower(-0.3))
                .onFalse(
                        intake.reset(Units.Degrees.zero().mutableCopy())
                                .alongWith(intake.setAnglePower(0)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> pathIndex = 0),
                Commands.runOnce(
                        () -> {
                            pathPlannerAuto = autos.get(autoChooser.getSelected());
                            pathPoints =
                                    PathPlannerAuto.getPathGroupFromAutoFile(
                                                    autoChooser.getSelected())
                                            .stream()
                                            .map(
                                                    (path) ->
                                                            path.getAllPathPoints()
                                                                    .get(
                                                                            path.getAllPathPoints()
                                                                                            .size()
                                                                                    - 1))
                                            .toList();
                        }),
                new GalacticProxyCommand(() -> pathPlannerAuto));
    }
}
