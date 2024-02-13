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
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.Utils;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.List;
import java.util.Set;

public class ShootState implements ScoreState {
    private static Shooter shooter;
    private static Hood hood;
    private Translation2d speakerPose;
    private InterpolatingDouble distanceToSpeaker = new InterpolatingDouble(0.0);
    private PoseEstimation poseEstimation;
    private Pose2d botPose;
    private List<Translation2d> optimalPoints;
    private Translation2d optimalTranslation;
    private double optimalRotation;
    private static boolean inBounds;

    public ShootState() {
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        poseEstimation = new PoseEstimation(SwerveDrive.getInstance());
    }

    private Command rotate() {
        return SwerveDrive.getInstance()
                .turnCommand(
                        optimalRotation, ScoreStateConstants.TURN_TOLERANCE.in(Units.Rotations));
    }

    private void updateInBounds() {
        double poseX = botPose.getX();
        InterpolatingDouble poseY = new InterpolatingDouble(botPose.getY());
        if (isRed()) {
            inBounds = poseX >= ScoreStateConstants.RED_BOUNDS_MAP.getInterpolated(poseY).value;
        } else {
            inBounds = poseX <= ScoreStateConstants.BLUE_BOUNDS_MAP.getInterpolated(poseY).value;
        }
    }

    private Command setShooter() {
        return shooter.setVelocity(
                () ->
                        Units.RotationsPerSecond.of(
                                        ShooterConstants.VELOCITY_BY_DISTANCE.getInterpolated(
                                                        distanceToSpeaker)
                                                .value)
                                .mutableCopy());
    }

    private Command setHood() {
        return hood.setAngle(
                () ->
                        Units.Degrees.of(
                                        Math.atan(
                                                ScoreStateConstants.SHOOTER_TO_SPEAKER_HEIGHT
                                                        / distanceToSpeaker.value))
                                .mutableCopy());
    }

    @Override
    public Command calculateTargets() {
        return Commands.runOnce(
                () -> {
                    botPose = poseEstimation.getEstimatedPose();
                    updateInBounds();
                    if (isRed()) {
                        speakerPose = ScoreStateConstants.SPEAKER_POSE_RED;
                        optimalPoints = ScoreStateConstants.OPTIMAL_POINTS_SHOOT_RED;
                    } else {
                        speakerPose = ScoreStateConstants.SPEAKER_POSE_BLUE;
                        optimalPoints = ScoreStateConstants.OPTIMAL_POINTS_SHOOT_BLUE;
                    }
                    optimalTranslation = botPose.getTranslation().nearest(optimalPoints);
                    optimalRotation =
                            Utils.calcRotationToTranslation(optimalTranslation, speakerPose)
                                    .getRotations();
                    distanceToSpeaker.value = Utils.getDistanceFromPoint(speakerPose, botPose);
                });
    }

    @Override
    public Command prepareSubsystems() {
        return setShooter()
                .alongWith(setHood())
                .onlyIf(
                        () ->
                                Math.abs(botPose.getX() - speakerPose.getX())
                                        <= ShooterConstants.MAX_WARMUP_DISTANCE)
                .alongWith(CommandGroups.getInstance().retractGrillevator());
    }

    @Override
    public Command driveToClosestOptimalPoint() {
        return Commands.defer(
                () -> {
                    updateInBounds();
                    if (inBounds) {
                        optimalTranslation = botPose.getTranslation();
                        return rotate();
                    }
                    optimalTranslation = botPose.getTranslation().nearest(optimalPoints);

                    return AutoBuilder.pathfindToPose(
                                    new Pose2d(optimalTranslation, new Rotation2d(optimalRotation)),
                                    Constants.AUTO_CONSTRAINTS)
                            .andThen(rotate());
                },
                Set.of(SwerveDrive.getInstance()));
    }

    @Override
    public Command score() {
        return Commands.sequence(
                Commands.parallel(setShooter(), setHood(), driveToClosestOptimalPoint()),
                Commands.runOnce(() -> SwerveDrive.getInstance().lock()),
                CommandGroups.getInstance().feedShooter());
    }
}
