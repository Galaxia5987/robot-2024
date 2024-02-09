package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.scoreStates.ScoreState;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.Supplier;

public class CommandGroups {
    private static CommandGroups INSTANCE;
    private final Intake intake;
    private final Gripper gripper;
    private final Elevator elevator;
    private final Shooter shooter;
    private final Conveyor conveyor;

    public static boolean override;

    private CommandGroups() {
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        elevator = Elevator.getInstance();
        shooter = Shooter.getInstance();
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
                gripper.setWristPosition(CommandGroupsConstants.WRIST_BASE_ANGLE));
    }

    public Command feed() {
        return conveyor.feed()
                .alongWith(
                        Commands.waitUntil(conveyor::readyToFeed)
                                .andThen(gripper.setRollerPower(GripperConstants.OUTTAKE_POWER)));
    }

    public Command intake() {
        return Commands.sequence(
                retractGrillevator(),
                Commands.parallel(intake.intake(), gripper.intake()).until(gripper::hasNote),
                Commands.parallel(intake.stop(), gripper.setRollerPower(0)));
    }

    public Command scoreTrap() {
        return Commands.sequence(
                gripper.setWristPosition(CommandGroupsConstants.WRIST_TRAP_ANGLE),
                gripper.setRollerPower(GripperConstants.TRAP_POWER));
    }

    public Command outtakeGripper() {
        return elevator.setHeight(CommandGroupsConstants.OUTTAKE_HEIGHT)
                .onlyIf(gripper::isGripperInsideRobot)
                .andThen(gripper.outtake());
    }

    public Command outtakeShooter() {
        return retractGrillevator()
                .andThen(
                        Commands.parallel(
                                feed(), shooter.setVelocity(() -> ShooterConstants.OUTTAKE_POWER)));
    }
}
