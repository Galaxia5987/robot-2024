package lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import java.util.Comparator;
import java.util.List;

public class Utils {
    public static final double EPSILON = 1e-9;

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(double a, double b, double maxError) {
        return Math.abs(a - b) <= maxError;
    }

    public static boolean speedsEpsilonEquals(ChassisSpeeds speeds) {
        return epsilonEquals(speeds.vxMetersPerSecond, 0)
                && epsilonEquals(speeds.vyMetersPerSecond, 0)
                && epsilonEquals(speeds.omegaRadiansPerSecond, 0);
    }

    public static Pose3d pose2dToPose3d(Pose2d pose) {
        return new Pose3d(
                pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
    }

    /**
     * Averages ambiguity of estimated poses using a harmonic average. Can be from different targets
     * in vision module, or between module.
     *
     * @param ambiguities the ambiguities to average.
     * @return the average of the ambiguities.
     */
    public static double averageAmbiguity(List<Double> ambiguities) {
        return 1.0 / ambiguities.stream().map((num) -> 1.0 / num).reduce(0.0, Double::sum);
    }

    public static double normalize(double angleRadians) {
        while (angleRadians < 0) {
            angleRadians += 2 * Math.PI;
        }
        return angleRadians % (2 * Math.PI);
    }

    public static Rotation2d normalize(Rotation2d angle) {
        return Rotation2d.fromRadians(normalize(angle.getRadians()));
    }

    public static Measure<Angle> normalize(Measure<Angle> angle) {
        return Units.Radians.of(normalize(angle.in(Units.Radians)));
    }

    public static double getDistanceFromPoint(Translation2d point, Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(point);
    }

    public static Pose2d calcClosestPose(List<Pose2d> points, Pose2d robotPose) {
        return points.stream()
                .min(
                        Comparator.comparingDouble(
                                point -> getDistanceFromPoint(point.getTranslation(), robotPose)))
                .orElse(null);
    }

    public static Rotation2d calcRotationToTranslation(
            Translation2d currentTranslation, Translation2d destinationTranslation) {
        return new Rotation2d(
                destinationTranslation.getX() - currentTranslation.getX(),
                destinationTranslation.getY() - currentTranslation.getY());
    }
}
