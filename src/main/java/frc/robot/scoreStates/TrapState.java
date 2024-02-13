package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Set;

public class TrapState implements ScoreState {

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    var optimalPoints =
                            isRed()
                                    ? ScoreStateConstants.OPTIMAL_POINTS_TRAP_RED
                                    : ScoreStateConstants.OPTIMAL_POINTS_TRAP_BLUE;
                    Pose2d optimalPose =
                            SwerveDrive.getInstance().getBotPose().nearest(optimalPoints);
                    return AutoBuilder.pathfindToPose(optimalPose, Constants.AUTO_CONSTRAINTS);
                },
                Set.of(SwerveDrive.getInstance()));
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
