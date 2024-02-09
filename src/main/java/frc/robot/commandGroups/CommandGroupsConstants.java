package frc.robot.commandGroups;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;

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
            Units.Degrees.of(180).mutableCopy(); // TODO: replace with actual value
    public static final MutableMeasure<Angle> WRIST_ANGLE_AMP_FORWARD =
            Units.Degrees.of(0).mutableCopy(); // TODO: replace with actual value;
    public static final MutableMeasure<Angle> WRIST_ANGLE_AMP_BACKWARDS =
            Units.Degrees.of(120).mutableCopy(); // TODO: replace with actual value;
    public static final MutableMeasure<Angle> WRIST_TRAP_ANGLE =
            Units.Degrees.of(120).mutableCopy(); // TODO: replace with actual value
    public static final MutableMeasure<Angle> WRIST_TOLERANCE =
            Units.Degrees.of(4).mutableCopy(); // TODO: replace with actual value
}
