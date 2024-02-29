package frc.robot.scoreStates;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroupsConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;

public class ClimbState implements ScoreState { // TODO: fix class
    private static Elevator elevator;

    public ClimbState() {
        elevator = Elevator.getInstance();
    }

    @Override
    public Command prepareSubsystems() {
        return elevator.setHeight(CommandGroupsConstants.START_CLIMB_HEIGHT);
    }

    @Override
    public Command score(Optional<CommandXboxController> driveController) {
        return Commands.sequence(
                Commands.runOnce(() -> SwerveDrive.getInstance().lock()),
                elevator.setHeight(CommandGroupsConstants.END_CLIMB_HEIGHT));
    }
}
