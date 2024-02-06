package frc.robot.scoreStates;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
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

    public static final List<Pose2d> OPTIMAL_POINTS_TRAP_BLUE =
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    public static final List<Pose2d> OPTIMAL_POINTS_TRAP_RED =
            OPTIMAL_POINTS_TRAP_BLUE.stream()
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
    public static final double TURN_TOLERANCE = 0.01; // Rotations
}
