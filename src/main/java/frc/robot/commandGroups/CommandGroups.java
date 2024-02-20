package frc.robot.commandGroups;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CommandGroups {
    private static CommandGroups INSTANCE;
    private final Intake intake;
    private final Gripper gripper;
    private final Elevator elevator;
    private final Shooter shooter;
    private final Hood hood;
    private final Conveyor conveyor;
    private SwerveDrive swerveDrive;
    private boolean override;

    private CommandGroups() {
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        elevator = Elevator.getInstance();
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

    public void toggleOverride() {
        override = !override;
    }

    public Command retractGrillevator() {
        return Commands.parallel(
                        elevator.setHeight(CommandGroupsConstants.MIN_HEIGHT),
                        gripper.setRollerAndWrist(CommandGroupsConstants.WRIST_BASE_ANGLE, 0))
                .withName("retractGrillevator");
    }

    public Command feed() {
        return conveyor.feed()
                .alongWith(
                        Commands.waitUntil(conveyor::readyToFeed)
                                .andThen(
                                        gripper.setRollerAndWrist(
                                                GripperConstants.INTAKE_ANGLE.mutableCopy(),
                                                GripperConstants.OUTTAKE_POWER)))
                .withName("feed");
    }

    public Command feedWithWait(BooleanSupplier otherReady) {
        return conveyor.feed()
                .alongWith(
                        Commands.waitUntil(
                                        () -> conveyor.readyToFeed() && otherReady.getAsBoolean())
                                .andThen(
                                        gripper.setRollerAndWrist(
                                                GripperConstants.INTAKE_ANGLE.mutableCopy(),
                                                GripperConstants.INTAKE_POWER)))
                .withName("feedWithWait");
    }

    public Command feedShooter() {
        return feedWithWait(() -> shooter.atSetpoint() && hood.atSetpoint())
                .withName("feedShooter");
    }

    public Command intake() {
        return Commands.repeatingSequence(
                        // retractGrillevator(),
                        Commands.parallel(
                                intake.intake(),
                                gripper.intake(),
                                Commands.waitUntil(gripper::hasNote)
                                        .andThen(Commands.none()))) // TODO: replace null with leds
                // mode
                .withName("intake");
    }

    public Command setShooter() {
        var distanceToSpeaker =
                new InterpolatingDouble(PoseEstimation.getInstance().getDistanceToSpeaker());
        return Commands.repeatingSequence(
                shooter.setVelocity(
                                () ->
                                        Units.RotationsPerSecond.of(
                                                        ShooterConstants.VELOCITY_BY_DISTANCE
                                                                .getInterpolated(distanceToSpeaker)
                                                                .value)
                                                .mutableCopy())
                        .until(shooter::atSetpoint));
    }

    public Command setScoringSystems() {
        return Commands.parallel(setHood(), setShooter());
    }

    public Command setHood() {
        var distanceToSpeaker =
                new InterpolatingDouble(PoseEstimation.getInstance().getDistanceToSpeaker());
        return Commands.repeatingSequence(
                hood.setAngle(
                                () ->
                                        Units.Degrees.of(
                                                        HoodConstants.ANGLE_BY_DISTANCE
                                                                .getInterpolated(distanceToSpeaker)
                                                                .value)
                                                .mutableCopy())
                        .until(hood::atSetpoint));
    }

    public Command outtakeGripper() {
        return elevator.setHeight(CommandGroupsConstants.OUTTAKE_HEIGHT)
                .onlyIf(gripper::isGripperNearFoldedPosition)
                .andThen(gripper.outtake())
                .withName("outtakeGripper");
    }

    public Command outtakeShooter() {
        return retractGrillevator()
                .andThen(
                        Commands.parallel(
                                feed(), shooter.setVelocity(() -> ShooterConstants.OUTTAKE_POWER)))
                .withName("outtakeShooter");
    }

    public Command shootAndConvey(Supplier<MutableMeasure<Velocity<Angle>>> velocity) {
        return shooter.setVelocity(velocity).alongWith(conveyor.setVelocity(velocity));
    }
}
