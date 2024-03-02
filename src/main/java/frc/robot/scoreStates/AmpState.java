package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.commandGroups.CommandGroupsConstants;
import frc.robot.lib.Utils;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Set;

public class AmpState implements ScoreState {
    private Climb elevator;
    private Gripper gripper;
    private boolean isAmpingForward = true;
    private boolean hasTimeToTurnGripper;
    private Translation2d ampPose;
    private double distance;
    private double robotRotation;
    private Rotation2d ampRotation;

    public AmpState() {
        elevator = Climb.getInstance();
        gripper = Gripper.getInstance();
    }

    @Override
    public Command calculateTargets() {
        return Commands.runOnce(
                () -> {
                    isAmpingForward = true;
                    if (ScoreState.isRed()) {
                        ampPose = ScoreStateConstants.AMP_POSE_RED;
                    } else {
                        ampPose = ScoreStateConstants.AMP_POSE_BLUE;
                    }
                    ampRotation = ScoreStateConstants.AMP_ROTATION_NORMAL;
                    Pose2d botPose = SwerveDrive.getInstance().getBotPose();
                    robotRotation = botPose.getRotation().getRadians();
                    distance = Utils.getDistanceFromPoint(ampPose, botPose);
                    hasTimeToTurnGripper =
                            distance
                                    > ScoreStateConstants.MIN_DISTANCE_TO_TURN_GRIPPER.in(
                                            Units.Meters);
                    if ((gripper.isReversed() && robotRotation < 0) || hasTimeToTurnGripper) {
                        isAmpingForward = false;
                        ampRotation = ScoreStateConstants.AMP_ROTATION_REVERSE;
                    }
                });
    }

    @Override
    public Command prepareSubsystems() {
        return Commands.none();
    }

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () ->
                        AutoBuilder.pathfindToPose(
                                new Pose2d(ampPose, ampRotation), Constants.AUTO_CONSTRAINTS),
                Set.of(SwerveDrive.getInstance()));
    }

    @Override
    public Command score() {
        return Commands.sequence(
                driveToClosestOptimalPoint(),
                Commands.runOnce(() -> SwerveDrive.getInstance().lock()),
                Commands.parallel(
                        elevator.setHeight(CommandGroupsConstants.MAX_HEIGHT),
                        Commands.either(
                                gripper.setRollerAndWrist(
                                        GripperConstants.AMP_POWER_NORMAL,
                                        CommandGroupsConstants.WRIST_ANGLE_AMP_FORWARD),
                                gripper.setRollerAndWrist(
                                        GripperConstants.AMP_POWER_REVERSE,
                                        CommandGroupsConstants.WRIST_ANGLE_AMP_BACKWARDS),
                                () -> isAmpingForward)));
    }

    @Override
    public Command finalizeScore() {
        return CommandGroups.getInstance().retractGrillevator();
    }
}
