package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GripperState;
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
    private final GripperState gripperState;

    public static boolean override;

    private CommandGroups() {
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        elevator = Elevator.getInstance();
        shooter = Shooter.getInstance();
        conveyor = Conveyor.getInstance();
        gripperState = new GripperState();
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
                gripper.setWristPosition(GripperConstants.WRIST_BASE_ANGLE));
    }

    public Command feed() {
        return conveyor.feed()
                .andThen(
                        gripper.setRollerPower(GripperConstants.OUTTAKE_POWER)
                                .onlyIf(conveyor::readyToFeed));
    }

    public Command intake() {
        return Commands.sequence(
                retractGrillevator(),
                Commands.parallel(intake.intake(), gripper.intake()).until(gripper::hasNote),
                Commands.parallel(intake.stop(), gripper.setRollerPower(0)));
    }

    public Command scoreTrap() {
        return Commands.sequence(
                gripper.setWristPosition(GripperConstants.WRIST_TRAP_ANGLE),
                gripper.setRollerPower(GripperConstants.TRAP_POWER));
    }

    public Command outtakeGripper() {
        return elevator.setHeight(CommandGroupsConstants.OUTTAKE_HEIGHT)
                .onlyIf(gripperState::isGripperInsideRobot)
                .andThen(gripper.outtake());
    }

    public Command outtakeShooter() {
        return retractGrillevator()
                .andThen(
                        Commands.parallel(
                                feed(), shooter.setVelocity(() -> ShooterConstants.OUTTAKE_POWER)));
    }

    public Command scoreCommandInit(Supplier<ScoreState> currentState) {
        return currentState.get().initializeCommand();
    }

    public Command scoreSubsystemInit(Supplier<ScoreState> currentState) {
        return currentState.get().initializeSubsystems();
    }

    public Command scoreExecute(Supplier<ScoreState> currentState) {
        return currentState.get().score();
    }
}
