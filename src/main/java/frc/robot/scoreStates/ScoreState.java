package frc.robot.scoreStates;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface ScoreState {
    default boolean isRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    default Command initializeCommand() {
        return Commands.none();
    }

    Command initializeSubsystems();

    Command driveToClosestOptimalPoint();

    Command score();
}
