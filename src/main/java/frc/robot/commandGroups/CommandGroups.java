package frc.robot.commandGroups;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.BooleanSupplier;

public class CommandGroups {
    private static CommandGroups INSTANCE;
    private final Intake intake;
    private final Gripper gripper;
    private final Elevator elevator;
    private final Shooter shooter;
    private final Hood hood;
    private final Conveyor conveyor;
    private final LEDs leds;
    private boolean override;

    private CommandGroups() {
        leds = new LEDs(0, 60);
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        elevator = Elevator.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        conveyor = Conveyor.getInstance();

        leds.setPrimary(Color.kAliceBlue);
        leds.setSecondary(Color.kOrangeRed);
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
                .andThen(gripper.setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.4))
                .withName("feedWithWait");
    }

    public Command intake() {
        return Commands.parallel(
                        intake.intake(),
                        gripper.setRollerPower(0.3)
                                .until(gripper::hasNote)
                                .andThen(gripper.setRollerPower(0))
                                .alongWith(leds.solidSecondary(1, 60)))
                .withName("intake");
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

    public Command shootAndConvey(MutableMeasure<Velocity<Angle>> velocity) {
        return shooter.setVelocity(velocity).alongWith(conveyor.setVelocity(velocity));
    }

    public Command stopHoodShooterConveyorGripper() {
        return Commands.parallel(
                hood.setAngle(Units.Degrees.of(114).mutableCopy()),
                shooter.setVelocity(Units.RotationsPerSecond.zero().mutableCopy()),
                conveyor.stop(),
                gripper.setRollerPower(0));
    }

    public Command stopAllSubsystems() {
        return stopHoodShooterConveyorGripper().alongWith(intake.stop());
    }

    public Command grillevatorBit() {
        return Commands.sequence(
                elevator.manualElevator(() -> -0.1).withTimeout(0.11),
                elevator.setHeight(CommandGroupsConstants.MAX_HEIGHT),
                gripper.setRollerAndWrist(0.3, CommandGroupsConstants.WRIST_ANGLE_AMP_FORWARD),
                Commands.waitSeconds(3),
                retractGrillevator());
    }

    public Command intakeBit() {
        return Commands.sequence(intake().withTimeout(3), intake.stop());
    }

    // TODO: create shooterBit, allBits commandGroups
}
