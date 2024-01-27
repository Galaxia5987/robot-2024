package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class ShooterConstants {
    public static final double GEAR_RATIO_TOP = 1.0;
    public static final double GEAR_RATIO_BOTTOM = 1.0;
    public static final double SETPOINT_TOLERANCE_TOP = 0.05; // Percentages
    public static final double SETPOINT_TOLERANCE_BOTTOM = 0.05; // Percentages
    public static final double MOMENT_OF_INERTIA_TOP = 0.08;
    public static final double MOMENT_OF_INERTIA_BOTTOM = 0.08;

    public static final LoggedTunableNumber TOP_kP = new LoggedTunableNumber("Shooter/Top kP");
    public static final LoggedTunableNumber TOP_kI = new LoggedTunableNumber("Shooter/Top kI");
    public static final LoggedTunableNumber TOP_kD = new LoggedTunableNumber("Shooter/Top kD");
    public static final LoggedTunableNumber TOP_kS = new LoggedTunableNumber("Shooter/Top kS");
    public static final LoggedTunableNumber TOP_Kv = new LoggedTunableNumber("Shooter/Top Kv");
    public static final LoggedTunableNumber TOP_Ka = new LoggedTunableNumber("Shooter/Top Ka");

    public static final LoggedTunableNumber BOTTOM_kP =
            new LoggedTunableNumber("Shooter/Bottom kP");
    public static final LoggedTunableNumber BOTTOM_kI =
            new LoggedTunableNumber("Shooter/Bottom kI");
    public static final LoggedTunableNumber BOTTOM_kD =
            new LoggedTunableNumber("Shooter/Bottom kD");
    public static final LoggedTunableNumber BOTTOM_kS =
            new LoggedTunableNumber("Shooter/Bottom kS");
    public static final LoggedTunableNumber BOTTOM_Kv =
            new LoggedTunableNumber("Shooter/Bottom Kv");
    public static final LoggedTunableNumber BOTTOM_Ka =
            new LoggedTunableNumber("Shooter/Bottom Ka");

    public static void initialize(Constants.Mode mode) {
        switch (mode) {
            case REAL:
                TOP_kP.initDefault(10.0);
                TOP_kI.initDefault(0.0);
                TOP_kD.initDefault(0.0);
                TOP_kS.initDefault(0.0);
                TOP_Kv.initDefault(0.112);
                TOP_Ka.initDefault(0.0);
                BOTTOM_kP.initDefault(10.0);
                BOTTOM_kI.initDefault(0.0);
                BOTTOM_kD.initDefault(0.0);
                BOTTOM_kS.initDefault(0.0);
                BOTTOM_Kv.initDefault(0.112);
                BOTTOM_Ka.initDefault(0.0);
            case SIM:
            case REPLAY:
                TOP_kP.initDefault(10.0);
                TOP_kI.initDefault(0.0);
                TOP_kD.initDefault(0.0);
                TOP_kS.initDefault(0.0);
                TOP_Kv.initDefault(0.112);
                TOP_Ka.initDefault(0.0);

                BOTTOM_kP.initDefault(10.0);
                BOTTOM_kI.initDefault(0.0);
                BOTTOM_kD.initDefault(0.0);
                BOTTOM_kS.initDefault(0.0);
                BOTTOM_Kv.initDefault(0.112);
                BOTTOM_Ka.initDefault(0.0);
        }
    }

    public static MutableMeasure<Velocity<Angle>> stopPower =
            Units.RotationsPerSecond.zero().mutableCopy();
}
