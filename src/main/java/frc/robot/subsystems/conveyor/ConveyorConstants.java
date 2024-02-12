package frc.robot.subsystems.conveyor;

import edu.wpi.first.units.*;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class ConveyorConstants {

    public static final MutableMeasure<Velocity<Angle>> FEED_VELOCITY =
            Units.RotationsPerSecond.of(10).mutableCopy();
    public static final MutableMeasure<Dimensionless> SETPOINT_TOLERANCE =
            Units.Value.of(0.05).mutableCopy();
    public static final MutableMeasure<Velocity<Angle>> STOP_VELOCITY =
            Units.RotationsPerSecond.of(0).mutableCopy();
    public static final double GEAR_RATIO = 30;
    public static LoggedTunableNumber KP = new LoggedTunableNumber("Conveyor/kP");
    public static LoggedTunableNumber KI = new LoggedTunableNumber("Conveyor/kI");
    public static LoggedTunableNumber KD = new LoggedTunableNumber("Conveyor/kD");
    public static LoggedTunableNumber KS = new LoggedTunableNumber("Conveyor/kS");
    public static LoggedTunableNumber KV = new LoggedTunableNumber("Conveyor/kV");
    public static LoggedTunableNumber KA = new LoggedTunableNumber("Conveyor/kA");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                KP.initDefault(0);
                KI.initDefault(0.1);
                KD.initDefault(0);
                KS.initDefault(0);
                KV.initDefault(0);
                KA.initDefault(0);
                break;
            case SIM:
            case REPLAY:
            default:
                KP.initDefault(0);
                KI.initDefault(0);
                KD.initDefault(0);
                KS.initDefault(0);
                KV.initDefault(1.87);
                KA.initDefault(0);
                break;
        }
    }
}
