package frc.robot.scoreStates;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShootState implements ScoreState {
    private final Shooter shooter;
    private final Hood hood;
    private final PoseEstimation poseEstimation;
    private final CommandGroups commandGroups;
    private final SwerveDrive swerveDrive;

    @Getter
    private final MutableMeasure<Velocity<Angle>> shooterVelocity =
            RotationsPerSecond.zero().mutableCopy();

    @Getter private final MutableMeasure<Angle> hoodAngle = Degrees.zero().mutableCopy();
    @Getter private final MutableMeasure<Angle> swerveAngle = Rotations.zero().mutableCopy();

    @Setter
    private Measure<Distance> maxWarmupDistance =
            Meters.of(100.0); // Arbitrary number larger than possible

    @Setter private Measure<Distance> maxShootingDistance = Meters.of(10.5);

    public ShootState() {
        shooter = Shooter.getInstance();
        poseEstimation = PoseEstimation.getInstance();
        commandGroups = CommandGroups.getInstance();
        swerveDrive = SwerveDrive.getInstance();
        hood = Hood.getInstance();
    }

    @AutoLogOutput(key = "ReadyToShoot")
    public boolean readyToShoot() {
        return poseEstimation.getDistanceToSpeaker() < maxShootingDistance.in(Meters)
                && hood.atSetpoint()
                && shooter.atSetpoint();
    }

    private Command swerveControllerAdjustment(Optional<CommandXboxController> driveController) {
        return swerveDrive.driveAndAdjust(
                getSwerveAngle(),
                () ->
                        driveController
                                .map(commandXboxController -> -commandXboxController.getLeftY())
                                .orElseThrow(),
                () ->
                        -driveController
                                .map(commandXboxController -> -commandXboxController.getLeftX())
                                .orElseThrow(),
                0.1);
    }

    private Command overrideAutoRotation() {
        return Commands.runOnce(
                () ->
                        PPHolonomicDriveController.setRotationTargetOverride(
                                () -> Optional.of(new Rotation2d(getSwerveAngle()))));
    }

    public Command feedShooter() {
        return commandGroups.feedWithWait(this::readyToShoot).withName("feedShooter");
    }

    @Override
    public Command calculateTargets() {
        return Commands.runOnce(
                () -> {
                    double distanceToTarget = poseEstimation.getDistanceToSpeaker();

                    if (distanceToTarget < maxWarmupDistance.in(Meters)) {
                        shooterVelocity.mut_replace(
                                ShooterConstants.VELOCITY_BY_DISTANCE.getInterpolated(
                                                new InterpolatingDouble(distanceToTarget))
                                        .value,
                                RotationsPerSecond);

                        hoodAngle.mut_replace(
                                HoodConstants.ANGLE_BY_DISTANCE.getInterpolated(
                                                new InterpolatingDouble(distanceToTarget))
                                        .value,
                                Degrees);
                    } else {
                        shooterVelocity.mut_replace(0, RotationsPerSecond);
                        hoodAngle.mut_replace(114, Degrees);
                    }

                    var toSpeaker = poseEstimation.getPoseRelativeToSpeaker();
                    swerveAngle
                            .mut_replace(
                                    Math.atan2(toSpeaker.getY(), toSpeaker.getX()) - Math.PI,
                                    Radians)
                            .mut_plus(-2, Degrees);
                });
    }

    public Command prepareSubsystems() {
        return Commands.parallel(
                hood.setAngle(getHoodAngle()), commandGroups.shootAndConvey(getShooterVelocity()));
    }

    @Override
    public Command score(Optional<CommandXboxController> driveController, boolean isAuto) {
        return Commands.either(
                        overrideAutoRotation(),
                        swerveControllerAdjustment(driveController).alongWith(prepareSubsystems()),
                        () -> isAuto)
                .until(this::readyToShoot)
                .alongWith(feedShooter());
    }
}
