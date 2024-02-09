package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.swerve.SwerveDrive;
import java.util.Set;
import lib.Utils;

public class AmpState implements ScoreState {
    private Elevator elevator;
    private Gripper gripper;
    private boolean isAmpingForward = true;
    private boolean hasTimeToTurnGripper;
    private Translation2d ampPose;
    private double distance;
    private double robotRotation;
    private Rotation2d ampRotation;

    public AmpState() {
        elevator = Elevator.getInstance();
        gripper = Gripper.getInstance();
    }

    @Override
    public Command initializeCommand() {
        return Commands.runOnce(
                () -> {
                    isAmpingForward = true;
                    if (isRed()) {
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
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () ->
                        AutoBuilder.pathfindToPose(
                                new Pose2d(ampPose, ampRotation), Constants.AUTO_CONSTRAINTS),
                Set.of(SwerveDrive.getInstance()));
    }

    @Override
    public Command initializeSubsystem() {
        return Commands.defer(
                () ->
                        Commands.parallel(
                                elevator.setHeight(ElevatorConstants.MAX_HEIGHT),
                                gripper.setWristPosition(
                                        isAmpingForward
                                                ? GripperConstants.WRIST_ANGLE_AMP_FORWARD
                                                        .mutableCopy()
                                                : GripperConstants.WRIST_ANGLE_AMP_BACKWARDS
                                                        .mutableCopy())),
                Set.of(gripper, elevator));
    }

    @Override
    public Command score() {
        return driveToClosestOptimalPoint()
                .andThen(
                        Commands.defer(
                                () ->
                                        gripper.setRollerPower(
                                                isAmpingForward
                                                        ? GripperConstants.AMP_POWER_NORMAL
                                                        : GripperConstants.AMP_POWER_REVERSE),
                                Set.of(gripper)));
    }
}
