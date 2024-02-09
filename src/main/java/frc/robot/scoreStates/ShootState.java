package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.PoseEstimation;
import frc.robot.commandGroups.CommandGroups;
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
    private Translation2d speakerPose;
    private double distanceToSpeaker;
    private Pose2d botPose;
    private List<Translation2d> optimalPoints;
    private Translation2d optimalTranslation;
    private double optimalRotation;
    private static boolean inBounds;

    private double shooterToSpeakerHeight =
            ScoreStateConstants.SPEAKER_TARGET_HEIGHT
                    - ShooterConstants.SHOOTER_HEIGHT.in(Units.Meters);

    public ShootState() {
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
    }

    private Command rotate() {
        return SwerveDrive.getInstance()
                .turnCommand(
                        optimalRotation, ScoreStateConstants.TURN_TOLERANCE.in(Units.Rotations));
    }

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    if (inBounds) {
                        optimalTranslation = botPose.getTranslation();
                        return rotate();
                    }
                    optimalTranslation = botPose.getTranslation().nearest(optimalPoints);

                    return AutoBuilder.pathfindToPose(
                            new Pose2d(optimalTranslation, new Rotation2d(optimalRotation)),
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
                                < ScoreStateConstants.RED_BOUNDS_MAP.get(botPose.getY()).value) {
                            inBounds = false;
                        }
                        speakerPose = ScoreStateConstants.SPEAKER_POSE_RED;
                        optimalPoints = ScoreStateConstants.OPTIMAL_POINTS_SHOOT_RED;
                    } else {
                        if (botPose.getX()
                                > ScoreStateConstants.BLUE_BOUNDS_MAP.get(botPose.getY()).value) {
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
    public Command initializeSubsystems() {
        shooterToSpeakerHeight =
                ScoreStateConstants.SPEAKER_TARGET_HEIGHT
                        - ShooterConstants.SHOOTER_HEIGHT.in(Units.Meters);
        return Commands.parallel(
                shooter.setVelocity(
                        () ->
                                Units.RotationsPerSecond.of(
                                                ShooterConstants.VELOCITY_BY_DISTANCE.get(
                                                                distanceToSpeaker)
                                                        .value)
                                        .mutableCopy()),
                hood.setAngle(
                        () ->
                                Units.Degrees.of(
                                                Math.atan(
                                                        shooterToSpeakerHeight / distanceToSpeaker))
                                        .mutableCopy()),
                CommandGroups.getInstance().retractGrillevator());
    }

    @Override
    public Command score() {
        return Commands.sequence(
                driveToClosestOptimalPoint(),
                rotate(),
                Commands.parallel(
                        shooter.setVelocity(
                                () ->
                                        Units.RotationsPerSecond.of(
                                                        ShooterConstants.VELOCITY_BY_DISTANCE.get(
                                                                        distanceToSpeaker)
                                                                .value)
                                                .mutableCopy()),
                        hood.setAngle(
                                () ->
                                        Units.Degrees.of(
                                                        Math.atan(
                                                                shooterToSpeakerHeight
                                                                        / distanceToSpeaker))
                                                .mutableCopy())),
                CommandGroups.getInstance()
                        .feed()
                        .onlyIf(() -> shooter.atSetpoint() && hood.atSetpoint()));
    }
}
