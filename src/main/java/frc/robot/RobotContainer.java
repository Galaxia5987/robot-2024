package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
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
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOReal;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.Set;

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

    private CommandGroups commandGroups;
    private ScoreState currentState;
    private final ShootState shootState;
    private final AmpState ampState;
    private final ClimbState climbState;

    private final PoseEstimation poseEstimation;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
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
//                elevatorIO = new ElevatorIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                intakeIO = new IntakeIOSim();
                conveyorIO = new ConveyorIOSim();
                gripperIO = new GripperIOSim();
                hoodIO = new HoodIOSim();
                shooterIO = new ShooterIOSim();
//                elevatorIO = new ElevatorIOSim();
                break;
        }
        Intake.initialize(intakeIO);
        Conveyor.initialize(conveyorIO);
//        Elevator.initialize(elevatorIO);
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

        SmartDashboard.putNumber("Shooter Velocity", 90);
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
                        () -> 0.6 * -xboxController.getRightX(),
                        0.15,
                        () -> true));
    }

    private void configureButtonBindings() {
        xboxController
                .y()
                .whileTrue(Commands.defer(() -> AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(13.1, 6.26), new Rotation2d()),
                        Constants.AUTO_CONSTRAINTS), Set.of(swerveDrive)));
        xboxController
                .rightBumper()
                .whileTrue(
                        Commands.parallel(
                                hood.setAngle(
                                        () -> HoodKinematics.getAngleFromDistance(
                                                Units.Meters.of(Robot.distanceToSpeaker)
                                        ).mutableCopy()),
                                commandGroups.shootAndConvey(() -> Units.RotationsPerSecond.of(80).mutableCopy())))
                .onFalse(
                        Commands.sequence(
                                gripper.setRollerPower(0.7).withTimeout(1),
                                Commands.parallel(
                                        hood.setAngle(() -> Units.Degrees.of(114).mutableCopy()),
                                        conveyor.stop(),
                                        shooter.stop(),
                                        gripper.setRollerPower(0))));
        xboxController.leftBumper().whileTrue(
                        Commands.parallel(
                                intake.intake(),
                                gripper.setRollerPower(0.1)
                        )
                ).onFalse(
                        Commands.parallel(
                                intake.stop(),
                                gripper.setRollerPower(0)
                        )
                );
        xboxController.rightTrigger().onTrue(Commands.runOnce(() -> intake.reset(Units.Degrees.zero())));
        xboxController.x().onTrue(Commands.runOnce(swerveDrive::resetPose));
        xboxController.b().onTrue(Commands.runOnce(swerveDrive::resetGyro));
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
