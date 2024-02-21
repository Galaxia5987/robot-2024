package frc.robot.scoreStates;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface ScoreState {
    static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    default Command calculateTargets() {
        return Commands.none();
    }

    Command prepareSubsystems();

    Command driveToClosestOptimalPoint();

    Command score();

    default Command finalizeScore() {
        return Commands.none();
    }
}
