package frc.robot.states;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import java.util.Arrays;
import lib.Utils;

public class ShootState implements ScoreState {

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    Pose2d optimalPose =
                            Utils.calcOptimalPose(
                                    Arrays.asList(Constants.OPTIMAL_POINTS_SHOOT),
                                    SwerveDrive.getInstance().getBotPose());
                    return AutoBuilder.pathfindToPose(optimalPose, Constants.AUTO_CONSTRAINTS);
                },
                DTOP_REQUIREMENTS);
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
