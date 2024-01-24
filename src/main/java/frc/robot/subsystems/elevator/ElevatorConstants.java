package frc.robot.subsystems.elevator;

import edu.wpi.first.units.*;
import lib.webconstants.LoggedTunableNumber;

public class ElevatorConstants { // TODO: check real values
    public static final double MECHANISM_WIDTH = 0; //[m]
    public static final double MECHANISM_HEIGHT = 0; //[m]
    public static final double GEAR_RATIO = 1 / 12;
    public static final double DRUM_RADIUS = 0.02; //[m]

    public static final MutableMeasure<Distance> STARTING_HEIGHT = Units.Meters.of(0).mutableCopy(); //[m]
    public static final MutableMeasure<Distance> MIN_HEIGHT = Units.Meters.of(0).mutableCopy(); //[m]
    public static final MutableMeasure<Distance> MAX_HEIGHT = Units.Meters.of(0).mutableCopy(); //[m]

    public static final LoggedTunableNumber KP = new LoggedTunableNumber("kp", 0);
    public static final LoggedTunableNumber KI = new LoggedTunableNumber("ki", 0);
    public static final LoggedTunableNumber KD = new LoggedTunableNumber("kd", 0);
}
