package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.swerve.SwerveDrive;
import java.util.Set;

public class ClimbState implements ScoreState {
    private static Elevator elevator;

    public ClimbState() {
        elevator = Elevator.getInstance();
    }

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    var optimalPoints =
                            isRed()
                                    ? ScoreStateConstants.OPTIMAL_POINTS_CLIMB_RED
                                    : ScoreStateConstants.OPTIMAL_POINTS_CLIMB_BLUE;
                    Pose2d optimalPose =
                            SwerveDrive.getInstance().getBotPose().nearest(optimalPoints);
                    return AutoBuilder.pathfindToPose(optimalPose, Constants.AUTO_CONSTRAINTS);
                },
                Set.of(SwerveDrive.getInstance()));
    }

    public Command initializeCommand() {
        return elevator.setHeight(ElevatorConstants.STARTING_CLIMB_HEIGHT);
    }

    @Override
    public Command initializeSubsystem() {
        return Commands.none();
    }

    @Override
    public Command score() {
        return Commands.parallel(
                driveToClosestOptimalPoint(),
                elevator.setHeight(ElevatorConstants.ENDING_CLIMB_HEIGHT));
    }
}
