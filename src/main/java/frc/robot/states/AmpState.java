package frc.robot.states;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import java.util.Arrays;
import java.util.HashSet;
import lib.Utils;

public class AmpState implements ScoreState {

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    Pose2d optimalPose =
                            Utils.calcOptimalPose( // TODO: make the robot not turn all the way
                                    Arrays.asList(Constants.OPTIMAL_POINTS_TRAP),
                                    SwerveDrive.getInstance().getBotPose());
                    return AutoBuilder.pathfindToPose(optimalPose, Constants.AUTO_CONSTRAINTS);
                },
                new HashSet<>() {
                    {
                        add(SwerveDrive.getInstance());
                    }
                });
    }

    @Override
    public Command stateInitialize() {
        return null;
    }

    @Override
    public Command score() {
        return null;
    }
}
