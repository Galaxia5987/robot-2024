package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.PoseEstimation;
import frc.robot.Robot;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.swerve.SwerveDrive;
import java.util.List;
import java.util.Set;
import lib.Utils;

public class ShootState implements ScoreState {
    private static Shooter shooter;
    private static Hood hood;
    private static Gripper gripper;
    private Translation2d speakerPose;
    private static boolean inBounds;
    private Translation2d optimalTranslation;
    private Translation2d optimalRotation;

    public ShootState() {
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        gripper = Gripper.getInstance();
    }

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    Pose2d botPose = PoseEstimation.getInstance().getEstimatedPose();
                    List<Translation2d> optimalPoints;
                    if (isRed()) {
                        speakerPose = ScoreStateConstants.SPEAKER_POSE_RED;
                        optimalPoints = ScoreStateConstants.OPTIMAL_POINTS_SHOOT_RED;
                    } else {
                        speakerPose = ScoreStateConstants.SPEAKER_POSE_BLUE;
                        optimalPoints = ScoreStateConstants.OPTIMAL_POINTS_SHOOT_BLUE;
                    }
                    if (Utils.getDistanceFromPoint(speakerPose, botPose)
                            < ScoreStateConstants.MAX_SHOOTING_DISTANCE.in(Units.Meters)) {
                        optimalTranslation = botPose.getTranslation();
                        return SwerveDrive.getInstance()
                                .turnCommand(
                                        Utils.calcRotationToTranslation(
                                                        optimalTranslation, speakerPose)
                                                .getRotations(),
                                        ScoreStateConstants.TURN_TOLERANCE.in(Units.Rotations));
                    }
                    optimalTranslation = botPose.getTranslation().nearest(optimalPoints);

                    return AutoBuilder.pathfindToPose(
                            new Pose2d(
                                    optimalTranslation,
                                    Utils.calcRotationToTranslation(
                                            optimalTranslation, speakerPose)),
                            Constants.AUTO_CONSTRAINTS);
                },
                Set.of(SwerveDrive.getInstance()));
    }

    @Override
    public Command initializeCommand() {
        var poseEstimation = PoseEstimation.getInstance().getEstimatedPose();
        return Commands.run(()-> {
            if (isRed()) {
                if (poseEstimation.getX() < ScoreStateConstants.redBoundsMap.get(poseEstimation.getY()).value) {
                    inBounds = false;
                }
            } else {
                if (poseEstimation.getX() > ScoreStateConstants.blueBoundsMap.get(poseEstimation.getY()).value) {
                    inBounds = false;
                }
            }

        });
    }

    @Override
    public Command initializeSubsystem() {
        return Commands.parallel(
                shooter.setVelocity(
                        () ->
                                Units.RotationsPerSecond.of(
                                                ShooterConstants.interpolationMap.get(
                                                                Utils.getDistanceFromPoint(
                                                                        speakerPose,
                                                                        PoseEstimation.getInstance()
                                                                                .getEstimatedPose()))
                                                        .value)
                                        .mutableCopy()),
                hood.setAngle(
                        () ->
                                Units.Degrees.of(
                                                Math.atan(
                                                        (Constants.SPEAKER_TARGET_POSE.getY()
                                                                        - ShooterConstants
                                                                                .SHOOTER_HEIGHT)
                                                                / Utils.getDistanceFromPoint(
                                                                        speakerPose,
                                                                        PoseEstimation.getInstance()
                                                                                .getEstimatedPose())))
                                        .mutableCopy()),
                CommandGroups.getINSTANCE().elevatorGripperMinPosition());
    }

    @Override
    public Command score() {
        return Commands.sequence(
                driveToClosestOptimalPoint(), gripper.setRollerPower(GripperConstants.SHOOT_POWER));
    }
}
