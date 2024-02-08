package frc.robot.scoreStates;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import lib.math.interpolation.InterpolatingDouble;
import lib.math.interpolation.InterpolatingDoubleMap;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

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
    public static final Translation2d AMP_POSE_RED = GeometryUtil.flipFieldPosition(AMP_POSE_BLUE);
    public static final Translation2d SPEAKER_POSE_BLUE = new Translation2d(0, 5.547_944_2);
    public static final Translation2d SPEAKER_POSE_RED =
            GeometryUtil.flipFieldPosition(SPEAKER_POSE_BLUE);
    public static final Rotation2d AMP_ROTATION_NORMAL = new Rotation2d(Math.toRadians(90));
    public static final Rotation2d AMP_ROTATION_REVERSE = new Rotation2d(Math.toRadians(-90));
    public static final Measure<Distance> MAX_SHOOTING_DISTANCE = Units.Meters.of(3);
    public static final Measure<Distance> MIN_DISTANCE_TO_TURN_GRIPPER = Units.Meters.of(3);
    public static final Measure<Angle> TURN_TOLERANCE = Units.Rotations.of(0.01); // Rotations

    public static final Transform3d SPEAKER_TARGET_POSE = // TODO: check real value
            new Transform3d(0.0, 0.0, 0.0, new Rotation3d());

    public static final InterpolatingDoubleMap blueBoundsMap = new InterpolatingDoubleMap(6);
    public static final InterpolatingDoubleMap redBoundsMap = new InterpolatingDoubleMap(6);

    static {
        blueBoundsMap.putAll(
            new HashMap<>(){{
                for (int i = 0; i < OPTIMAL_POINTS_SHOOT_BLUE.size(); i++) {
                    put(
                            new InterpolatingDouble(OPTIMAL_POINTS_SHOOT_BLUE.get(i).getY()),
                            new InterpolatingDouble(OPTIMAL_POINTS_SHOOT_BLUE.get(i).getX()));
                }
            }}
        );

        redBoundsMap.putAll(
                new HashMap<>(){{
                    for (int i = 0; i < OPTIMAL_POINTS_SHOOT_RED.size(); i++) {
                        put(
                                new InterpolatingDouble(OPTIMAL_POINTS_SHOOT_RED.get(i).getY()),
                                new InterpolatingDouble(OPTIMAL_POINTS_SHOOT_RED.get(i).getX()));
                    }
                }}
        );
    }
}
