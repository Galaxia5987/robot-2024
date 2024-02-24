package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.Utils;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.Getter;
import lombok.Setter;

public class ShootingManager {

    private static ShootingManager INSTANCE = null;

    private final PoseEstimation poseEstimation;
    private final SwerveDrive swerveDrive;
    private final Hood hood;
    private final Shooter shooter;

    @Getter
    private final MutableMeasure<Velocity<Angle>> shooterCommandedVelocity =
            RotationsPerSecond.zero().mutableCopy();

    @Getter private final MutableMeasure<Angle> hoodCommandedAngle = Degrees.zero().mutableCopy();

    @Getter
    private final MutableMeasure<Angle> swerveCommandedAngle = Rotations.zero().mutableCopy();

    @Setter
    private Measure<Distance> maxWarmupDistance =
            Meters.of(100.0); // Arbitrary number larger than possible

    @Setter private Measure<Distance> maxShootingDistance = Meters.of(2.5);

    private ShootingManager() {
        poseEstimation = PoseEstimation.getInstance();
        swerveDrive = SwerveDrive.getInstance();
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
    }

    public static ShootingManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShootingManager();
        }
        return INSTANCE;
    }

    public boolean readyToShoot() {
        return poseEstimation.getDistanceToSpeaker() < maxShootingDistance.in(Meters)
                && hood.atSetpoint()
                && shooter.atSetpoint()
                && Utils.epsilonEquals(
                        swerveDrive.getYaw().getDegrees(), swerveCommandedAngle.in(Degrees), 3);
    }

    public void updateCommandedState() {
        double distanceToTarget = poseEstimation.getDistanceToSpeaker();

        if (distanceToTarget < maxWarmupDistance.in(Meters)) {
            shooterCommandedVelocity.mut_replace(
                    ShooterConstants.VELOCITY_BY_DISTANCE.getInterpolated(
                                    new InterpolatingDouble(distanceToTarget))
                            .value,
                    RotationsPerSecond);

            hoodCommandedAngle.mut_replace(
                    HoodConstants.ANGLE_BY_DISTANCE.getInterpolated(
                                    new InterpolatingDouble(distanceToTarget))
                            .value,
                    Degrees);
        } else {
            shooterCommandedVelocity.mut_replace(0, RotationsPerSecond);
            hoodCommandedAngle.mut_replace(114, Degrees);
        }

        var toSpeaker = poseEstimation.getPoseRelativeToSpeaker();
        swerveCommandedAngle.mut_replace(
                Math.atan2(toSpeaker.getY(), toSpeaker.getX()) - Math.PI, Radians);
    }

    public void updateHoodChassisCompensation() {
        double rotationalVelocity = swerveDrive.getCurrentSpeeds().omegaRadiansPerSecond;
        double hoodAngle = hood.getAngle().in(Radians);
        double radialAcceleration =
                rotationalVelocity
                        * rotationalVelocity
                        * (HoodConstants.AXIS_DISTANCE_TO_CENTER.in(Meters)
                                - Math.cos(hoodAngle) * HoodConstants.CM_RADIUS.in(Meters));
        double effectiveAcceleration = swerveDrive.getAcceleration() - radialAcceleration;

        double tangentialAcceleration = effectiveAcceleration * Math.cos(hoodAngle - Math.PI / 2);
        double torque =
                tangentialAcceleration
                        * HoodConstants.MASS.in(Kilograms)
                        * HoodConstants.CM_RADIUS.in(Meters);
        hood.setChassisCompensationTorque(torque);
    }
}
