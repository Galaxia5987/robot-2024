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
                    Pose2d ampPose = Constants.AMP_POSE;
                    Pose2d botPose = SwerveDrive.getInstance().getBotPose();
                    double distance = Utils.getDistanceFromPoint(ampPose, botPose);
                    double robotRotation = botPose.getRotation().getRadians();
                    boolean isGripperReversed =
                            false; // TODO: replace with actual gripper.isForward
                    boolean hasTimeToTurnGripper =
                            distance > Constants.MIN_DISTANCE_TO_TURN_GRIPPER.in(Units.Meters);

                    if ((isGripperReversed && robotRotation < 0) || hasTimeToTurnGripper) {
                        ampPose =
                                ampPose.transformBy(
                                        new Transform2d(0, 0, new Rotation2d(Math.toRadians(180))));
                    }
                    return AutoBuilder.pathfindToPose(ampPose, Constants.AUTO_CONSTRAINTS);
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
