package frc.robot.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.swerve.SwerveDrive;
import java.util.HashSet;

interface ScoreState {
    HashSet<Subsystem> DTOP_REQUIREMENTS =
            new HashSet<>() {
                {
                    add(SwerveDrive.getInstance());
                }
            };

    Command driveToClosestOptimalPoint();

    Command stateInitialize();

    Command score();
}
