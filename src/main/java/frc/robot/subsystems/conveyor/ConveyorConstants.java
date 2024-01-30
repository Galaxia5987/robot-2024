package frc.robot.subsystems.conveyor;

import edu.wpi.first.units.*;
import lib.webconstants.LoggedTunableNumber;

public class ConveyorConstants {

    public static final MutableMeasure<Velocity<Angle>> FEED_VELOCITY =
            Units.RotationsPerSecond.of(10).mutableCopy();
    public static final MutableMeasure<Dimensionless> SETPOINT_TOLERANCE =
            Units.Value.of(0.05).mutableCopy();
    public static final double GEAR_RATIO = 0;
    public static LoggedTunableNumber KP = new LoggedTunableNumber("Conveyor/kP");
    public static LoggedTunableNumber KI = new LoggedTunableNumber("Conveyor/kI");
    public static LoggedTunableNumber KD = new LoggedTunableNumber("Conveyor/kD");
    public static LoggedTunableNumber KS = new LoggedTunableNumber("Conveyor/kS");
    public static LoggedTunableNumber KV = new LoggedTunableNumber("Conveyor/kV");
    public static LoggedTunableNumber KA = new LoggedTunableNumber("Conveyor/kA");


}
