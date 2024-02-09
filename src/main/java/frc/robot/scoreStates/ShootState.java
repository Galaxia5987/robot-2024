package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.PoseEstimation;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.subsystems.gripper.Gripper;
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
    private double optimalRotation;
    private List<Translation2d> optimalPoints;
    private double distanceToSpeaker;
    private Pose2d botPose;

    public ShootState() {
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        gripper = Gripper.getInstance();
    }

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    if (inBounds) {
                        optimalTranslation = botPose.getTranslation();
                        return SwerveDrive.getInstance()
                                .turnCommand(
                                        optimalRotation,
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
        botPose = PoseEstimation.getInstance().getEstimatedPose();
        return Commands.run(
                () -> {
                    if (isRed()) {
                        if (botPose.getX()
                                < ScoreStateConstants.redBoundsMap.get(botPose.getY()).value) {
                            inBounds = false;
                        }
                        speakerPose = ScoreStateConstants.SPEAKER_POSE_RED;
                        optimalPoints = ScoreStateConstants.OPTIMAL_POINTS_SHOOT_RED;
                    } else {
                        if (botPose.getX()
                                > ScoreStateConstants.blueBoundsMap.get(botPose.getY()).value) {
                            inBounds = false;
                        }
                        speakerPose = ScoreStateConstants.SPEAKER_POSE_BLUE;
                        optimalPoints = ScoreStateConstants.OPTIMAL_POINTS_SHOOT_BLUE;
                    }
                    optimalTranslation = botPose.getTranslation().nearest(optimalPoints);
                    optimalRotation =
                            Utils.calcRotationToTranslation(optimalTranslation, speakerPose)
                                    .getRotations();
                    distanceToSpeaker = Utils.getDistanceFromPoint(speakerPose, botPose);
                });
    }

    @Override
    public Command initializeSubsystem() {
        return Commands.parallel(
                shooter.setVelocity(
                        () ->
                                Units.RotationsPerSecond.of(
                                                ShooterConstants.interpolationMap.get(
                                                                distanceToSpeaker)
                                                        .value)
                                        .mutableCopy()),
                hood.setAngle(
                        () ->
                                Units.Degrees.of(
                                                Math.atan(
                                                        (ScoreStateConstants.SPEAKER_TARGET_HEIGHT
                                                                        - ShooterConstants
                                                                                .SHOOTER_HEIGHT
                                                                                .in(Units.Meters))
                                                                / distanceToSpeaker))
                                        .mutableCopy()),
                CommandGroups.getInstance().retractGrillevator());
    }

    @Override
    public Command score() {
        return Commands.sequence(
                driveToClosestOptimalPoint(),
                Commands.parallel(
                        shooter.setVelocity(
                                () ->
                                        Units.RotationsPerSecond.of(
                                                        ShooterConstants.interpolationMap.get(
                                                                        distanceToSpeaker)
                                                                .value)
                                                .mutableCopy()),
                        hood.setAngle(
                                () ->
                                        Units.Degrees.of(
                                                        Math.atan(
                                                                (ScoreStateConstants
                                                                                        .SPEAKER_TARGET_HEIGHT
                                                                                - ShooterConstants
                                                                                        .SHOOTER_HEIGHT
                                                                                        .in(
                                                                                                Units
                                                                                                        .Meters))
                                                                        / distanceToSpeaker))
                                                .mutableCopy())),
                CommandGroups.getInstance()
                        .feed()
                        .onlyIf(() -> shooter.atSetpoint() && hood.atSetpoint()));
    }
}
