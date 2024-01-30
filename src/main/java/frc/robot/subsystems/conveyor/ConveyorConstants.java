package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import lib.webconstants.LoggedTunableNumber;

public class ConveyorConstants {

    public static final MutableMeasure<Velocity<Angle>> FEED_VELOCITY =
            Units.RotationsPerSecond.of(10).mutableCopy();
    public static final MutableMeasure<Dimensionless> SETPOINT_TOLERANCE =
            Units.Value.of(0.05).mutableCopy();
    public static final double GEAR_RATIO = 0;
    public static LoggedTunableNumber KP = new LoggedTunableNumber("Conveyor/kP", 0);
    public static LoggedTunableNumber KI = new LoggedTunableNumber("Conveyor/kI", 0);
    public static LoggedTunableNumber KD = new LoggedTunableNumber("Conveyor/kD", 0);

    public static PIDController CONTROLLER = new PIDController(KP.get(), KI.get(), KD.get(), Timer.getFPGATimestamp());
}
