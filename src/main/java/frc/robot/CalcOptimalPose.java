package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.robotState.RobotState;
import frc.robot.swerve.SwerveDrive;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;

public class CalcOptimalPose {
    private Pose2d optimalPose;

    private double getDistanceFromPoint(Pose2d point, Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(point.getTranslation());
    }

    private Pose2d calcOptimalPose(List<Pose2d> points, Pose2d robotPose) {
        return points.stream()
                .min(Comparator.comparingDouble(point -> getDistanceFromPoint(point, robotPose)))
                .orElse(null);
    }

    public Command getClosestOptimalPoint() {
        Pathfinding.setPathfinder(new LocalADStar());
        return Commands.runOnce(
                        () ->
                                optimalPose =
                                        switch (RobotState.currentState) {
                                            case SHOOT -> calcOptimalPose(
                                                    List.of(Constants.OPTIMAL_POINTS_SHOOT),
                                                    SwerveDrive.getInstance().getBotPose());
                                            case TRAP -> calcOptimalPose(
                                                    List.of(Constants.OPTIMAL_POINTS_TRAP),
                                                    SwerveDrive.getInstance().getBotPose());
                                            case AMP -> Constants.AMP_POSE;
                                        })
                .andThen(
                        Commands.defer(
                                () ->
                                        AutoBuilder.pathfindToPose(
                                                optimalPose, Constants.autoConstraints)),
                new HashSet<>() {
                    {
                        add(SwerveDrive.getInstance());
                    }
                });
    }
}
