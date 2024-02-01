package frc.robot.subsystems.elevator;

import edu.wpi.first.units.*;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class ElevatorConstants { // TODO: check real values
    public static final double MECHANISM_WIDTH = 0; // [m]
    public static final double MECHANISM_HEIGHT = 0; // [m]
    public static final double GEAR_RATIO = 12.0;
    public static final double DRUM_RADIUS = 0.02; // [m]

    public static final MutableMeasure<Distance> STARTING_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Distance> MIN_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Distance> MAX_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]

    public static final LoggedTunableNumber KP = new LoggedTunableNumber("kp");
    public static final LoggedTunableNumber KI = new LoggedTunableNumber("ki");
    public static final LoggedTunableNumber KD = new LoggedTunableNumber("kd");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                KP.initDefault(0.1);
                KI.initDefault(0.0);
                KD.initDefault(0.0);
            case SIM:
            case REPLAY:
                KP.initDefault(0.2);
                KI.initDefault(0.0);
                KD.initDefault(0.0);
        }
    }
}
