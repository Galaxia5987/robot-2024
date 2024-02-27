package frc.robot.commandGroups;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
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

public class CommandGroups {
    private static CommandGroups INSTANCE;
    private final Intake intake;
    private final Gripper gripper;
    private final Elevator elevator;
    private final Shooter shooter;
    private final Hood hood;
    private final Conveyor conveyor;
    private boolean override;

    private CommandGroups() {
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        elevator = Elevator.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        conveyor = Conveyor.getInstance();

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
                        gripper.setRollerAndWrist(0, CommandGroupsConstants.WRIST_BASE_ANGLE))
                .withName("retractGrillevator");
    }

    public Command feed() {
        return conveyor.feed()
                .alongWith(
                        Commands.waitUntil(conveyor::readyToFeed)
                                .andThen(
                                        gripper.setRollerAndWrist(
                                                GripperConstants.OUTTAKE_POWER,
                                                GripperConstants.INTAKE_ANGLE.mutableCopy())))
                .withName("feed");
    }

    public Command feedWithWait(BooleanSupplier otherReady) {
        return Commands.waitUntil(otherReady)
                .andThen(gripper.setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.2))
                .withName("feedWithWait");
    }

    public Command feedShooter() {
        return feedWithWait(ShootingManager.getInstance()::readyToShoot).withName("feedShooter");
    }

    public Command intake() {
        return Commands.repeatingSequence(
                        // retractGrillevator(),
                        Commands.parallel(
                                intake.intake(),
                                gripper.setRollerPower(0.3)
                                        .until(gripper::hasNote)
                                        .andThen(
                                                gripper.setRollerPower(
                                                        0)))) // TODO: replace null with leds
                // mode
                .withName("intake");
    }

    public Command setShooter() {
        return Commands.repeatingSequence(
                shooter.setVelocity(ShootingManager.getInstance().getShooterCommandedVelocity())
                        .until(shooter::atSetpoint));
    }

    public Command setScoringSystems() {
        return Commands.parallel(setHood(), setShooter());
    }

    public Command setHood() {
        var distanceToSpeaker =
                new InterpolatingDouble(PoseEstimation.getInstance().getDistanceToSpeaker());
        return Commands.repeatingSequence(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle())
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
                                feed(), shooter.setVelocity(ShooterConstants.OUTTAKE_POWER)))
                .withName("outtakeShooter");
    }

    public Command shootAndConvey(
            MutableMeasure<Velocity<Angle>> topVelocity,
            MutableMeasure<Velocity<Angle>> bottomVelocity) {
        return shooter.setVelocity(topVelocity, bottomVelocity)
                .alongWith(conveyor.setVelocity(bottomVelocity));
    }

    public Command shootAndConvey(MutableMeasure<Velocity<Angle>> velocity) {
        return shooter.setVelocity(velocity).alongWith(conveyor.setVelocity(velocity));
    }

    public Command grillevatorBit() {
        return Commands.sequence(
                elevator.setHeight(CommandGroupsConstants.MAX_HEIGHT),
                gripper.setRollerAndWrist(0.3, CommandGroupsConstants.WRIST_ANGLE_AMP_FORWARD),
                Commands.waitSeconds(3),
                retractGrillevator());
    }

    public Command intakeBit() {
        return Commands.sequence(intake.intake(), Commands.waitSeconds(3), intake.stop());
    }

    public Command shooterBit() {
        return Commands.parallel(
                        shooter.setVelocity(Units.RotationsPerSecond.of(55).mutableCopy()),
                        hood.setAngle(Units.Degrees.of(70).mutableCopy()))
                .andThen(
                        Commands.sequence(
                                Commands.waitSeconds(3),
                                shooter.stop(),
                                hood.setAngle(Units.Degrees.of(114).mutableCopy())));
    }

    public Command shootAndIntake() {
        return Commands.parallel(gripper.setRollerPower(0.7), intake.intake());
    }

    public Command shootToAmp() {
        return shooter.setVelocity(
                        ShooterConstants.TOP_AMP_VELOCITY, ShooterConstants.BOTTOM_VELOCITY)
                .alongWith(hood.setAngle(HoodConstants.AMP_ANGLE))
                .until(shooter::atSetpoint)
                .andThen(gripper.setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.5))
                .andThen(gripper.setRollerPower(0))
                .alongWith(conveyor.setVelocity(ConveyorConstants.AMP_VELOCITY));
    }

    public Command allBits() {
        return Commands.sequence(
                intakeBit(),
                shooterBit(),
                grillevatorBit(),
                SwerveDrive.getInstance().checkSwerve());
    }
}
