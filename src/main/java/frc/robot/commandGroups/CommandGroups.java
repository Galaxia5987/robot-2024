package frc.robot.commandGroups;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class CommandGroups {
    private static CommandGroups INSTANCE;
    private final Intake intake;
    private final Gripper gripper;
    private final Climb climb;
    private final Shooter shooter;
    private final Hood hood;
    private final Conveyor conveyor;
    private final SwerveDrive swerveDrive;
    private boolean override;

    private CommandGroups() {
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        climb = Climb.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        conveyor = Conveyor.getInstance();
        swerveDrive = SwerveDrive.getInstance();
        override = false;
    }

    public static CommandGroups getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CommandGroups();
        }
        return INSTANCE;
    }

    public Command feedWithWait(BooleanSupplier otherReady) {
        return Commands.sequence(
                        Commands.waitSeconds(0.05),
                        Commands.waitUntil(otherReady),
                        gripper.setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.4))
                .withName("feedWithWait");
    }

    public Command feedShooter(BooleanSupplier isForceShooting) {
        return feedWithWait(
                        () ->
                                ShootingManager.getInstance().readyToShoot()
                                        || isForceShooting.getAsBoolean())
                .withName("feedShooter");
    }

    public Command intake(Command rumble) {
        return Commands.parallel(intake.intake(), gripper.setRollerPower(0.4))
                .until(gripper::hasNote)
                .andThen(Commands.parallel(intake.stop(), gripper.setRollerPower(0), rumble))
                .withName("intake");
    }

    public Command shootAndConvey(
            MutableMeasure<Velocity<Angle>> topVelocity,
            MutableMeasure<Velocity<Angle>> bottomVelocity) {
        return shooter.setVelocity(topVelocity, bottomVelocity)
                .alongWith(conveyor.setVelocity(bottomVelocity));
    }

    public Command shootAndConvey(
            MutableMeasure<Velocity<Angle>> velocity, boolean useConveyorMap) {
        return shooter.setVelocity(velocity)
                .alongWith(
                        conveyor.setVelocity(
                                useConveyorMap
                                        ? Units.RotationsPerSecond.of(50).mutableCopy()
                                        : ShootingManager.getInstance()
                                                .getConveyorCommandedVelocity()));
    }

    public Command intakeBit() {
        return Commands.sequence(intake(Commands.none()).withTimeout(3), intake.stop());
    }

    public Command shooterBit() {
        return shootToAmp()
                .andThen(
                        Commands.waitSeconds(3),
                        shooter.stop(),
                        conveyor.stop(),
                        hood.setAngle(Units.Degrees.of(114).mutableCopy()));
    }

    public Command shootAndIntake() {
        return Commands.parallel(gripper.setRollerPower(0.7), intake.intake());
    }

    public Command shootToAmp(CommandPS5Controller driveController) {
        return shooter.setVelocity(
                        ShooterConstants.TOP_AMP_VELOCITY, ShooterConstants.BOTTOM_AMP_VELOCITY)
                .alongWith(hood.setAngle(HoodConstants.AMP_ANGLE))
                .alongWith(conveyor.setVelocity(ConveyorConstants.AMP_VELOCITY))
                //                .alongWith(swerveDrive.driveAndAdjust( //TODO: check if this works
                //                        Units.Rotation.of(90).mutableCopy(),
                //                        () -> -driveController.getLeftY(),
                //                        () -> -driveController.getLeftX(),
                //                        0.1))
                .until(() -> shooter.atSetpoint() && hood.atSetpoint() && conveyor.atSetpoint())
                .andThen(gripper.setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.5))
                .andThen(gripper.setRollerPower(0));
    }

    public Command shootToTrap() { // TODO: remove from defer when calibrated
        return Commands.defer(
                () ->
                        Commands.parallel(
                                        swerveDrive.turnCommand(
                                                Units.Rotations.of(
                                                                CommandGroupsConstants.TRAP_POSE
                                                                        .getRotation()
                                                                        .getRotations())
                                                        .mutableCopy(),
                                                0.5 / 360.0),
                                        shooter.setVelocity(
                                                Units.RotationsPerSecond.of(
                                                                CommandGroupsConstants
                                                                        .TRAP_TOP_SHOOTER_VELOCITY
                                                                        .get())
                                                        .mutableCopy(),
                                                Units.RotationsPerSecond.of(
                                                                CommandGroupsConstants
                                                                        .TRAP_BOTTOM_SHOOTER_VELOCITY
                                                                        .get())
                                                        .mutableCopy()),
                                        conveyor.setVelocity(
                                                Units.RotationsPerSecond.of(
                                                                CommandGroupsConstants
                                                                        .TRAP_CONVEYOR_VELOCITY
                                                                        .get())
                                                        .mutableCopy()),
                                        hood.setAngle(
                                                Units.Degrees.of(
                                                                CommandGroupsConstants
                                                                        .TRAP_HOOD_ANGLE
                                                                        .get())
                                                        .mutableCopy()))
                                .until(
                                        () ->
                                                shooter.atSetpoint()
                                                        && hood.atSetpoint()
                                                        && conveyor.atSetpoint())
                                .andThen(
                                        gripper.setRollerPower(GripperConstants.INTAKE_POWER)
                                                .withTimeout(0.5))
                                .andThen(gripper.setRollerPower(0)),
                Set.of(shooter, hood, conveyor, gripper));
    }

    public Command dtopToTrap() {
        return Commands.defer(
                () -> {
                    var optimalPose = CommandGroupsConstants.TRAP_POSE;
                    return AutoBuilder.pathfindToPose(optimalPose, Constants.AUTO_CONSTRAINTS);
                },
                Set.of(swerveDrive));
    }

    public Command shootToSpeaker(CommandXboxController driveController) {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                shootAndConvey(ShootingManager.getInstance().getShooterCommandedVelocity(), true),
                swerveDrive.driveAndAdjust(
                        ShootingManager.getInstance().getSwerveCommandedAngle(),
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        0.1));
    }

    public Command shootToSpeaker(CommandPS5Controller driveController) {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                shootAndConvey(ShootingManager.getInstance().getShooterCommandedVelocity(), true),
                swerveDrive.driveAndAdjust(
                        ShootingManager.getInstance().getSwerveCommandedAngle(),
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        0.1));
    }

    public Command shootToSpeaker() {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                shootAndConvey(ShootingManager.getInstance().getShooterCommandedVelocity(), true));
    }

    public Command closeSpeakerWarmup() {
        return hood.setAngle(Units.Degrees.of(107).mutableCopy())
                .alongWith(shootAndConvey(Units.RotationsPerSecond.of(50).mutableCopy(), false));
    }

    public Command dtopClimb() {
        return Commands.defer(
                () -> {
                    var climbPoses =
                            new java.util.ArrayList<>(List.of(CommandGroupsConstants.CLIMB_POSES));
                    if (Constants.isRed()) {
                        climbPoses.replaceAll(GeometryUtil::flipFieldPose);
                    }
                    var optimalPose = swerveDrive.getBotPose().nearest(climbPoses);
                    return AutoBuilder.pathfindToPose(optimalPose, Constants.AUTO_CONSTRAINTS)
                            .andThen(
                                    swerveDrive.turnCommand(
                                            Units.Rotations.of(
                                                            optimalPose
                                                                    .getRotation()
                                                                    .getRotations())
                                                    .mutableCopy(),
                                            2.0 / 360.0));
                },
                Set.of(swerveDrive));
    }

    public Command stopShooting() {
        return Commands.parallel(
                hood.setAngle(Units.Degrees.of(114).mutableCopy()),
                shooter.stop(),
                conveyor.stop(),
                gripper.setRollerPower(0));
    }

    public Command allBits() {
        return Commands.sequence(intakeBit(), shooterBit().withTimeout(3));
    }
}
