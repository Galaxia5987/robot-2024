package frc.robot.scoreStates;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public interface ScoreState {
    default boolean isRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    Command initializeCommand();

    Command driveToClosestOptimalPoint();

    Command initializeSubsystem();

    Command score();
}
