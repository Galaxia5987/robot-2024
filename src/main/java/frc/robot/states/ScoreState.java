package frc.robot.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.swerve.SwerveDrive;
import java.util.Set;

public interface ScoreState {
    Set<Subsystem> DTOP_REQUIREMENTS = Set.of(SwerveDrive.getInstance());

    Command driveToClosestOptimalPoint();

    Command stateInitialize();

    Command score();
}
