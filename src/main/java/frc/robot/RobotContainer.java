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
import edu.wpi.first.wpilibj.GenericHID;
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
import frc.robot.subsystems.holder.Holder;
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
    private final Holder holder;
    private final Climb climb;
    private final SwerveDrive swerveDrive;
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandGroups commandGroups;
    private final SendableChooser<String> autoChooser;
    private final HashMap<String, PathPlannerAuto> autos = new HashMap<>();

    public int pathIndex = 0;
    private PathPlannerAuto pathPlannerAuto;
    private List<PathPoint> pathPoints =
            new ArrayList<>() {
                {
                    add(new PathPoint(new Translation2d()));
                }
            };
    private final Timer scoreTimer = new Timer();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ClimbIO climbIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                climbIO = new ClimbIOReal();
                break;
            case SIM:
                climbIO = new ClimbIOSim();
                break;
            case REPLAY:
            default:
                climbIO = new ClimbIO() {};
                break;
        }
        Climb.initialize(climbIO);
        Constants.initSwerve();

        swerveDrive = SwerveDrive.getInstance();
        climb = Climb.getInstance();
        holder = Holder.getInstance();
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
        NamedCommands.registerCommand(
                "followPathRotation",
                Commands.runOnce(() -> ShootingManager.getInstance().setShooting(false)));
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

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
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
                                        -(driverController.getLeftTriggerAxis() + 1) / 2
                                                + (driverController.getRightTriggerAxis() + 1) / 2,
                                        0.15)));
    }

    private void configureButtonBindings() {
        driverController.y().onTrue(Commands.runOnce(swerveDrive::resetGyro));
        driverController.rightBumper().whileTrue(holder.setPower(0.3))
                .onFalse(holder.setPower(0));
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
