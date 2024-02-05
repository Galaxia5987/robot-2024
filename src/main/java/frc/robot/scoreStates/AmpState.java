package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import lib.Utils;

public class AmpState implements ScoreState {

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    Translation2d ampPose;
                    Rotation2d ampRotation;
                    if (isRed()) {
                        ampPose = ScoreStateConstants.AMP_POSE_RED;
                        ampRotation = ScoreStateConstants.AMP_ROTATION_NORMAL_RED;
                    } else {
                        ampPose = ScoreStateConstants.AMP_POSE_BLUE;
                        ampRotation = ScoreStateConstants.AMP_ROTATION_NORMAL_BLUE;
                    }
                    Pose2d botPose = SwerveDrive.getInstance().getBotPose();
                    double robotRotation = botPose.getRotation().getRadians();
                    boolean isGripperReversed =
                            false; // TODO: replace with actual gripper.isForward
                    double distance = Utils.getDistanceFromPoint(ampPose, botPose);
                    boolean hasTimeToTurnGripper =
                            distance
                                    > ScoreStateConstants.MIN_DISTANCE_TO_TURN_GRIPPER.in(
                                            Units.Meters);

                    if ((isGripperReversed && robotRotation < 0) || hasTimeToTurnGripper) {
                        ampRotation = ScoreStateConstants.AMP_ROTATION_REVERSE_BLUE;
                    }
                    return AutoBuilder.pathfindToPose(
                            new Pose2d(ampPose, ampRotation), Constants.AUTO_CONSTRAINTS);
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
