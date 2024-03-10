package frc.robot.scoreStates;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap;
import java.util.List;
import java.util.stream.Collectors;

public class ScoreStateConstants {
    public static final List<Translation2d> OPTIMAL_POINTS_SHOOT_BLUE =
            List.of(
                    new Translation2d(0, 1000),
                    new Translation2d(1.97, 7.16),
                    new Translation2d(2.54, 3.06),
                    new Translation2d(4.16, 5.03),
                    new Translation2d(20, -1000));

    public static final List<Translation2d> OPTIMAL_POINTS_SHOOT_RED =
            OPTIMAL_POINTS_SHOOT_BLUE.stream()
                    .map(GeometryUtil::flipFieldPosition)
                    .collect(Collectors.toList());

    public static final List<Pose2d> OPTIMAL_POINTS_CLIMB_BLUE =
            List.of(
                    new Pose2d(4.35, 5.00, Rotation2d.fromDegrees(-47.47)),
                    new Pose2d(4.35, 3.27, Rotation2d.fromDegrees(61.38)),
                    new Pose2d(5.90, 4.13, Rotation2d.fromDegrees(-179.36)));
    public static final List<Pose2d> OPTIMAL_POINTS_CLIMB_RED =
            OPTIMAL_POINTS_CLIMB_BLUE.stream()
                    .map(GeometryUtil::flipFieldPose)
                    .collect(Collectors.toList());
    public static final Translation2d AMP_POSE_BLUE = new Translation2d(1.87, 7.64);
    public static final Translation2d AMP_POSE_RED = new Translation2d(14.67, 7.64);
    public static final Translation2d SPEAKER_POSE_BLUE = new Translation2d(0, 5.547_944_2);
    public static final Translation2d SPEAKER_POSE_RED =
            GeometryUtil.flipFieldPosition(SPEAKER_POSE_BLUE);
    public static final Rotation2d AMP_ROTATION_NORMAL = new Rotation2d(Math.toRadians(90));
    public static final Rotation2d AMP_ROTATION_REVERSE = new Rotation2d(Math.toRadians(-90));
    public static final MutableMeasure<Distance> MIN_DISTANCE_TO_TURN_GRIPPER =
            Units.Meters.of(3).mutableCopy();
    public static final MutableMeasure<Angle> TURN_TOLERANCE = Units.Degrees.of(5).mutableCopy();

    public static final double SHOOTER_TO_SPEAKER_HEIGHT = 0.0; // TODO: check real value

    private static InterpolatingDoubleMap createBoundsMap(List<Translation2d> optimalPoints) {
        InterpolatingDoubleMap map = new InterpolatingDoubleMap();
        optimalPoints.forEach(
                point ->
                        map.put(
                                new InterpolatingDouble(point.getY()),
                                new InterpolatingDouble(point.getX())));
        return map;
    }

    public static final InterpolatingDoubleMap BLUE_BOUNDS_MAP =
            createBoundsMap(OPTIMAL_POINTS_SHOOT_BLUE);
    public static final InterpolatingDoubleMap RED_BOUNDS_MAP =
            createBoundsMap(OPTIMAL_POINTS_SHOOT_RED);
}
