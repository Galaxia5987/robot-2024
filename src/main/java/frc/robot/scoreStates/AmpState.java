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
    private static Elevator elevator;
    private static Gripper gripper;
    private static boolean isAmpingForward = true;

    private AmpState() {
        elevator = Elevator.getInstance();
        gripper = Gripper.getInstance();
    }

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    Translation2d ampPose;
                    isAmpingForward = true;
                    if (isRed()) {
                        ampPose = ScoreStateConstants.AMP_POSE_RED;
                    } else {
                        ampPose = ScoreStateConstants.AMP_POSE_BLUE;
                    }
                    Rotation2d ampRotation;
                    ampRotation = ScoreStateConstants.AMP_ROTATION_NORMAL;
                    Pose2d botPose = SwerveDrive.getInstance().getBotPose();
                    double robotRotation = botPose.getRotation().getRadians();
                    double distance = Utils.getDistanceFromPoint(ampPose, botPose);
                    boolean hasTimeToTurnGripper =
                            distance
                                    > ScoreStateConstants.MIN_DISTANCE_TO_TURN_GRIPPER.in(
                                            Units.Meters);
                    if ((gripper.isReversed() && robotRotation < 0) || hasTimeToTurnGripper) {
                        isAmpingForward = false;
                        ampRotation = ScoreStateConstants.AMP_ROTATION_REVERSE;
                    }
                    return AutoBuilder.pathfindToPose(
                            new Pose2d(ampPose, ampRotation), Constants.AUTO_CONSTRAINTS);
                },
                Set.of(SwerveDrive.getInstance()));
    }

    @Override
    public Command stateInitialize() {
        return Commands.parallel(
                elevator.setHeight(ElevatorConstants.MAX_HEIGHT),
                gripper.setWristPosition(
                        isAmpingForward
                                ? GripperConstants.WRIST_FORWARD_AMP_POSE.mutableCopy()
                                : GripperConstants.WRIST_BACKWARDS_AMP.mutableCopy()));
    }

    @Override
    public Command score() {
        return driveToClosestOptimalPoint()
                .andThen(
                        gripper.setRollerPower(
                                isAmpingForward
                                        ? GripperConstants.AMP_POWER_NORMAL
                                        : GripperConstants.AMP_POWER_REVERSE));
    }
}
