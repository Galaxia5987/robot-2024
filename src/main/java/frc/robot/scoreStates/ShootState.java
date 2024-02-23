package frc.robot.scoreStates;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.Utils;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.List;
import java.util.Set;

public class ShootState implements ScoreState {
    private static boolean inBounds;
    private final Shooter shooter;
    private final Hood hood;
    private final Conveyor conveyor;
    private Translation2d speakerPose;
    private InterpolatingDouble distanceToSpeaker = new InterpolatingDouble(0.0);
    private PoseEstimation poseEstimation;
    private Pose2d botPose = new Pose2d();
    private List<Translation2d> optimalPoints;
    private Translation2d optimalTranslation;
    private double optimalRotation;

    public ShootState() {
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        conveyor = Conveyor.getInstance();
        poseEstimation = PoseEstimation.getInstance();
    }

    private Command rotate() {
        return SwerveDrive.getInstance()
                .turnCommand(
                        () -> Rotation2d.fromRotations(optimalRotation),
                        ScoreStateConstants.TURN_TOLERANCE.in(Units.Rotations));
    }

    private void updateInBounds() {
        double poseX = botPose.getX();
        InterpolatingDouble poseY = new InterpolatingDouble(botPose.getY());
        if (ScoreState.isRed()) {
            inBounds = poseX >= ScoreStateConstants.RED_BOUNDS_MAP.getInterpolated(poseY).value;
        } else {
            inBounds = poseX <= ScoreStateConstants.BLUE_BOUNDS_MAP.getInterpolated(poseY).value;
        }
    }

    public Command setShooter() {
        return shooter.setVelocity(ShootingManager.getInstance().getShooterCommandedVelocity())
                .until(shooter::atSetpoint);
    }

    public Command setHood() {
        return hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle())
                .until(hood::atSetpoint);
    }

    @Override
    public Command calculateTargets() {
        return Commands.runOnce(
                () -> {
                    botPose = poseEstimation.getEstimatedPose();
                    updateInBounds();
                    if (ScoreState.isRed()) {
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
                    if (DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                        optimalRotation =
                                Math.IEEEremainder(optimalRotation - Math.PI, Math.PI * 2);
                    }
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
        //                .alongWith(CommandGroups.getInstance().retractGrillevator())
        ;
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
                    calculateTargets();
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
        return Commands.repeatingSequence(
                Commands.parallel(
                        driveToClosestOptimalPoint()
                                .andThen(() -> SwerveDrive.getInstance().lock()),
                        setShooter(),
                        setHood(),
                        CommandGroups.getInstance().feedShooter()));
    }

    @Override
    public Command finalizeScore() {
        return Commands.parallel(
                shooter.stop(), hood.setAngle(HoodConstants.FOLDED_ANGLE), conveyor.stop());
    }
}
