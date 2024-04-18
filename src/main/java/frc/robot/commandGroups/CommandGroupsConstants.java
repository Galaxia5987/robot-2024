package frc.robot.commandGroups;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public class CommandGroupsConstants {
    // TODO: replace with actual values
    public static final MutableMeasure<Distance> STARTING_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Distance> MIN_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Distance> MAX_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Distance> START_CLIMB_HEIGHT =
            Units.Meters.of(0).mutableCopy();
    public static final MutableMeasure<Distance> END_CLIMB_HEIGHT =
            Units.Meters.of(0).mutableCopy();
    public static final MutableMeasure<Distance> OUTTAKE_HEIGHT = Units.Meters.of(0).mutableCopy();

    public static final MutableMeasure<Angle> WRIST_BASE_ANGLE =
            Units.Degrees.of(-80).mutableCopy(); // TODO: replace with actual value
    public static final MutableMeasure<Angle> WRIST_ANGLE_AMP_FORWARD =
            Units.Rotations.of(-1.396).mutableCopy(); // TODO: replace with actual value;
    public static final MutableMeasure<Angle> WRIST_MIN_BACKWARDS_ANGLE =
            Units.Degrees.of(90).mutableCopy();
    public static final MutableMeasure<Angle> WRIST_ANGLE_AMP_BACKWARDS =
            Units.Radians.of(2.15).mutableCopy(); // TODO: replace with actual value;

    private static final Pose2d CLIMB_TOP =
            new Pose2d(new Translation2d(4.39, 4.67), Rotation2d.fromDegrees(-57.72));
    private static final Pose2d CLIMB_MIDDLE =
            new Pose2d(new Translation2d(5.59, 4.09), Rotation2d.fromDegrees(180.00));
    private static final Pose2d CLIMB_BOTTOM =
            new Pose2d(new Translation2d(4.39, 3.46), Rotation2d.fromDegrees(57.72));
    public static final Pose2d[] CLIMB_POSES = {CLIMB_TOP, CLIMB_MIDDLE, CLIMB_BOTTOM};

    public static final LoggedTunableNumber TRAP_TOP_SHOOTER_VELOCITY =
            new LoggedTunableNumber("Trap/top_shooter_velocity", 30);
    public static final LoggedTunableNumber TRAP_BOTTOM_SHOOTER_VELOCITY =
            new LoggedTunableNumber("Trap/bottom_shooter_velocity", 60);
    public static final LoggedTunableNumber TRAP_CONVEYOR_VELOCITY =
            new LoggedTunableNumber("Trap/conveyor_velocity", 40);
    public static final LoggedTunableNumber TRAP_HOOD_ANGLE =
            new LoggedTunableNumber("Trap/hood_angle", 108);
    public static final Pose2d TRAP_POSE =
            new Pose2d(new Translation2d(12.38, 5.33), new Rotation2d(0.818));

    public static final Translation2d SUPER_POOP_POSE_BLUE = new Translation2d(0.28, 7.42);

    public static final Translation2d SUPER_POOP_POSE_RED =
            GeometryUtil.flipFieldPosition(SUPER_POOP_POSE_BLUE);
}
