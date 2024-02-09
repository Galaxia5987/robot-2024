package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GripperState;
import frc.robot.scoreStates.ScoreState;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
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

    public Command elevatorGripperMinPosition() {
        return Commands.parallel(
                elevator.setHeight(ElevatorConstants.MIN_HEIGHT),
                gripper.setWristPosition(GripperConstants.WRIST_FOLD_POSITION.mutableCopy()));
    }

    public Command intake() {
        return elevatorGripperMinPosition()
                .andThen(
                        Commands.parallel(intake.intake(), gripper.intake())
                                .until(gripper::hasNote)
                                .andThen(intake.stop(), gripper.setRollerPower(0)));
    }

    public Command trapScore() {
        return Commands.sequence(
                gripper.setWristPosition(GripperConstants.WRIST_TRAP_ANGLE.mutableCopy()),
                gripper.setRollerPower(GripperConstants.TRAP_POWER));
    }

    public Command gripperOuttake() {
        return elevator.setHeight(ElevatorConstants.OUTTAKE_HEIGHT)
                .onlyIf(gripperState::isGripperInsideRobot)
                .andThen(gripper.outtake());
    }

    public Command shooterOuttake() {
        return elevatorGripperMinPosition()
                .andThen(
                        Commands.parallel(
                                gripper.setRollerPower(GripperConstants.OUTTAKE_POWER),
                                conveyor.feed(),
                                shooter.setVelocity(
                                        () -> ShooterConstants.OUTTAKE_POWER.mutableCopy())));
    }

    public Command scoreCommandInit(Supplier<ScoreState> currentState) {
        return currentState.get().initializeCommand();
    }

    public Command scoreSubsystemInit(Supplier<ScoreState> currentState) {
        return currentState.get().initializeSubsystem();
    }

    public Command scoreExecute(Supplier<ScoreState> currentState) {
        return currentState.get().score();
    }

    public Command climb() {
        return null;
    }
}
