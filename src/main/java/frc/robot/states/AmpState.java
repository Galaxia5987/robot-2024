package frc.robot.states;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
                    Translation2d ampPose = Constants.AMP_POSE;
                    Rotation2d ampRotation = new Rotation2d(Math.toRadians(90));
                    Pose2d botPose = SwerveDrive.getInstance().getBotPose();
                    double robotRotation = botPose.getRotation().getRadians();
                    boolean isGripperReversed =
                            false; // TODO: replace with actual gripper.isForward
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                        ampPose = GeometryUtil.flipFieldPosition(ampPose);
                    }
                    double distance = Utils.getDistanceFromPoint(ampPose, botPose);
                    boolean hasTimeToTurnGripper =
                            distance > Constants.MIN_DISTANCE_TO_TURN_GRIPPER.in(Units.Meters);

                    if ((isGripperReversed && robotRotation < 0) || hasTimeToTurnGripper) {
                        ampRotation = new Rotation2d(Math.toRadians(-90));
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
