package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.Utils;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingManager {

    private static ShootingManager INSTANCE = null;

    private final PoseEstimation poseEstimation;
    private final SwerveDrive swerveDrive;
    private final Hood hood;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Vision vision;

    @Getter
    private final MutableMeasure<Velocity<Angle>> shooterCommandedVelocity =
            RotationsPerSecond.zero().mutableCopy();

    @Getter
    private final MutableMeasure<Velocity<Angle>> conveyorCommandedVelocity =
            RotationsPerSecond.zero().mutableCopy();

    @Getter private final MutableMeasure<Angle> hoodCommandedAngle = Degrees.zero().mutableCopy();

    @Getter
    private final MutableMeasure<Angle> swerveCommandedAngle = Rotations.zero().mutableCopy();

    @Setter
    private Measure<Distance> maxWarmupDistance =
            Meters.of(9.0); // Arbitrary number larger than possible

    @Setter private boolean lockShoot = false;

    @Setter private Measure<Distance> maxShootingDistance = Meters.of(10.5);

    private boolean isShooting = false;
    private final Debouncer useAlternateYaw = new Debouncer(0.1);
    private final Debouncer usePoseEstimation = new Debouncer(0.1);

    @Getter private double distanceToSpeaker = 0;

    public boolean useChassisCompensation = false;

    private ShootingManager() {
        poseEstimation = PoseEstimation.getInstance();
        swerveDrive = SwerveDrive.getInstance();
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
        conveyor = Conveyor.getInstance();
        vision = Vision.getInstance();
    }

    public static ShootingManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShootingManager();
        }
        return INSTANCE;
    }

    @AutoLogOutput(key = "Robot/ReadyToShoot")
    public boolean readyToShoot() {
        return poseEstimation.getDistanceToSpeaker() < maxShootingDistance.in(Meters)
                && hood.atSetpoint()
                && shooter.atSetpoint()
                && conveyor.atSetpoint()
                && (DriverStation.isAutonomous()
                        || Utils.epsilonEquals(
                                swerveCommandedAngle.in(Degrees),
                                swerveDrive.getOdometryYaw().getDegrees(),
                                2))
                && !lockShoot;
    }

    public void updateCommandedStateWithPoseEstimation() {
        var toSpeaker = poseEstimation.getPoseRelativeToSpeaker();
        distanceToSpeaker = toSpeaker.getNorm() * Utils.distanceToSpeakerVarianceFactor(toSpeaker);

        shooterCommandedVelocity.mut_replace(
                ShooterConstants.VELOCITY_BY_DISTANCE.getInterpolated(
                                new InterpolatingDouble(distanceToSpeaker))
                        .value,
                RotationsPerSecond);

        conveyorCommandedVelocity.mut_replace(
                ConveyorConstants.VELOCITY_BY_DISTANCE.getInterpolated(
                                new InterpolatingDouble(distanceToSpeaker))
                        .value,
                RotationsPerSecond);

        hoodCommandedAngle.mut_replace(
                HoodConstants.ANGLE_BY_DISTANCE.getInterpolated(
                                new InterpolatingDouble(distanceToSpeaker))
                        .value,
                Degrees);

        swerveCommandedAngle
                .mut_replace(Math.atan2(toSpeaker.getY(), toSpeaker.getX()) - Math.PI, Radians)
                .mut_minus(4, Degrees);
    }

    public void updateCommandedState() {
        var scoreParameters = vision.getScoreParameters();
        if (!usePoseEstimation.calculate(scoreParameters.isEmpty())) {
            distanceToSpeaker =
                    scoreParameters.stream()
                                    .mapToDouble(
                                            (param) ->
                                                    param.distanceToSpeaker()
                                                            * param.distanceVarianceFactor())
                                    .reduce(0, Double::sum)
                            / scoreParameters.size();

            shooterCommandedVelocity.mut_replace(
                    ShooterConstants.VELOCITY_BY_DISTANCE.getInterpolated(
                                    new InterpolatingDouble(distanceToSpeaker))
                            .value,
                    RotationsPerSecond);

            conveyorCommandedVelocity.mut_replace(
                    ConveyorConstants.VELOCITY_BY_DISTANCE.getInterpolated(
                                    new InterpolatingDouble(distanceToSpeaker))
                            .value,
                    RotationsPerSecond);

            hoodCommandedAngle.mut_replace(
                    HoodConstants.ANGLE_BY_DISTANCE.getInterpolated(
                                    new InterpolatingDouble(distanceToSpeaker))
                            .value,
                    Degrees);

            var yaws =
                    scoreParameters.stream()
                            .map(VisionIO.ScoreParameters::yaw)
                            .filter(Optional::isPresent)
                            .map(Optional::get)
                            .toList();
            Rotation2d yawToTarget;
            boolean useEstimation = useAlternateYaw.calculate(yaws.isEmpty());
            if (!useEstimation) {
                yawToTarget =
                        yaws.stream().reduce(new Rotation2d(), Rotation2d::plus).div(yaws.size());
            } else {
                yawToTarget =
                        scoreParameters.stream()
                                .map(VisionIO.ScoreParameters::alternateYaw)
                                .reduce(new Rotation2d(), Rotation2d::plus)
                                .div(scoreParameters.size());
            }

            swerveCommandedAngle
                    .mut_replace(
                            yawToTarget.getRotations()
                                    + swerveDrive.getOdometryYaw().getRotations(),
                            Rotations)
                    .mut_minus(4, Degrees);
        } else {
            updateCommandedStateWithPoseEstimation();
        }
    }

    public void updateHoodChassisCompensation() {
        double rotationalVelocity = swerveDrive.getCurrentSpeeds().omegaRadiansPerSecond;
        double hoodAngle = hood.getAngle().in(Radians);
        double radialAcceleration =
                rotationalVelocity
                        * rotationalVelocity
                        * (HoodConstants.AXIS_DISTANCE_TO_CENTER.in(Meters)
                                - Math.cos(hoodAngle) * HoodConstants.CM_RADIUS.in(Meters));
        double effectiveAcceleration = -swerveDrive.getAcceleration() - radialAcceleration;

        double tangentialAcceleration = effectiveAcceleration * Math.cos(hoodAngle - Math.PI / 2);
        double torque =
                tangentialAcceleration
                        * HoodConstants.MASS.in(Kilograms)
                        * HoodConstants.CM_RADIUS.in(Meters);

        Logger.recordOutput("Hood/TorqueChassisCompensation", torque);
        hood.setChassisCompensationTorque(torque);
    }

    public void setShooting(boolean shooting) {
        isShooting = shooting;
    }

    @AutoLogOutput(key = "Robot/isShooting")
    public boolean isShooting() {
        return isShooting;
    }
}
