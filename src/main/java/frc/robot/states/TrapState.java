package frc.robot.states;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import lib.Utils;

public class TrapState implements ScoreState {

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    var optimalPoints = Constants.OPTIMAL_POINTS_TRAP_BLUE;
                    if (isRed()) {
                        optimalPoints = Constants.OPTIMAL_POINTS_TRAP_RED;
                    }
                    Pose2d optimalPose =
                            Utils.calcOptimalPose(
                                    optimalPoints, SwerveDrive.getInstance().getBotPose());
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
