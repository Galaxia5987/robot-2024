package frc.robot.scoreStates;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.swerve.SwerveDrive;
import java.util.Set;

public interface ScoreState {
    default boolean isRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    Set<Subsystem> DTOP_REQUIREMENTS = Set.of(SwerveDrive.getInstance());

    Command driveToClosestOptimalPoint();

    Command stateInitialize();

    Command score();
}
