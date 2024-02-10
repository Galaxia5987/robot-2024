package frc.robot.scoreStates;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;
import lib.math.interpolation.InterpolatingDouble;
import lib.math.interpolation.InterpolatingDoubleMap;

public class ScoreStateConstants {
    public static final List<Translation2d> OPTIMAL_POINTS_SHOOT_BLUE =
            List.of(
                    new Translation2d(1.97, 7.16),
                    new Translation2d(2.54, 3.06),
                    new Translation2d(4.16, 5.03));
    public static final List<Translation2d> OPTIMAL_POINTS_SHOOT_RED =
            OPTIMAL_POINTS_SHOOT_BLUE.stream()
                    .map(GeometryUtil::flipFieldPosition)
                    .collect(Collectors.toList());

    public static final List<Pose2d> OPTIMAL_POINTS_CLIMB_BLUE =
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    public static final List<Pose2d> OPTIMAL_POINTS_CLIMB_RED =
            OPTIMAL_POINTS_CLIMB_BLUE.stream()
                    .map(GeometryUtil::flipFieldPose)
                    .collect(Collectors.toList());
    public static final Translation2d AMP_POSE_BLUE = new Translation2d(1.85, 7.49);
    public static final Translation2d AMP_POSE_RED =
            new Translation2d(0.0, 0.0); // TODO: add real value
    public static final Translation2d SPEAKER_POSE_BLUE = new Translation2d(0, 5.547_944_2);
    public static final Translation2d SPEAKER_POSE_RED =
            GeometryUtil.flipFieldPosition(SPEAKER_POSE_BLUE);
    public static final Rotation2d AMP_ROTATION_NORMAL = new Rotation2d(Math.toRadians(90));
    public static final Rotation2d AMP_ROTATION_REVERSE = new Rotation2d(Math.toRadians(-90));
    public static final MutableMeasure<Distance> MIN_DISTANCE_TO_TURN_GRIPPER =
            Units.Meters.of(3).mutableCopy();
    public static final MutableMeasure<Angle> TURN_TOLERANCE = Units.Degrees.of(5).mutableCopy();

    public static final double SPEAKER_TARGET_HEIGHT = 0.0; // TODO: check real value

    private static InterpolatingDoubleMap createBoundsMap(List<Translation2d> optimalPoints) {
        InterpolatingDoubleMap map = new InterpolatingDoubleMap();
        map.putAll(
                new HashMap<>() {
                    {
                        for (int i = 0; i < optimalPoints.size(); i++) {
                            put(
                                    new InterpolatingDouble(optimalPoints.get(i).getY()),
                                    new InterpolatingDouble(optimalPoints.get(i).getX()));
                        }
                    }
                });
        return map;
    }

    public static final InterpolatingDoubleMap BLUE_BOUNDS_MAP =
            createBoundsMap(OPTIMAL_POINTS_SHOOT_BLUE);
    public static final InterpolatingDoubleMap RED_BOUNDS_MAP =
            createBoundsMap(OPTIMAL_POINTS_SHOOT_RED);
}
