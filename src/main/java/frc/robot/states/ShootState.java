package frc.robot.states;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
                    var alliance = DriverStation.getAlliance();
                    var optimalPoints = Arrays.asList(Constants.OPTIMAL_POINTS_SHOOT);
                    if (!alliance.isEmpty()) {
                        if (alliance.get() == DriverStation.Alliance.Red) {
                            optimalPoints.forEach(GeometryUtil::flipFieldPose);
                        }
                    }
                    Pose2d optimalPose = // TODO: flip all the points in the list
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
