package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class ShooterConstants {
    public static final double GEAR_RATIO = 1.0;
    public static final double SETPOINT_TOLERANCE = 0.05; // Percentages
    public static final double MOMENT_OF_INERTIA = 0.08;

    public static final LoggedTunableNumber TOP_P = new LoggedTunableNumber("Shooter/Top P");
    public static final LoggedTunableNumber TOP_I = new LoggedTunableNumber("Shooter/Top I");
    public static final LoggedTunableNumber TOP_D = new LoggedTunableNumber("Shooter/Top D");
    public static final LoggedTunableNumber TOP_Ks = new LoggedTunableNumber("Shooter/Top Ks");
    public static final LoggedTunableNumber TOP_Kv = new LoggedTunableNumber("Shooter/Top Kv");
    public static final LoggedTunableNumber TOP_Ka = new LoggedTunableNumber("Shooter/Top Ka");

    public static final LoggedTunableNumber BOTTOM_P = new LoggedTunableNumber("Shooter/Bottom P");
    public static final LoggedTunableNumber BOTTOM_I = new LoggedTunableNumber("Shooter/Bottom I");
    public static final LoggedTunableNumber BOTTOM_D = new LoggedTunableNumber("Shooter/Bottom D");
    public static final LoggedTunableNumber BOTTOM_Ks =
            new LoggedTunableNumber("Shooter/Bottom Ks");
    public static final LoggedTunableNumber BOTTOM_Kv =
            new LoggedTunableNumber("Shooter/Bottom Kv");
    public static final LoggedTunableNumber BOTTOM_Ka =
            new LoggedTunableNumber("Shooter/Bottom Ka");

    public static void initialize(Constants.Mode mode) {
        switch (mode) {
            case REAL:
            case SIM:
            case REPLAY:
                TOP_P.initDefault(10.0);
                TOP_I.initDefault(0.0);
                TOP_D.initDefault(0.0);
                TOP_Ks.initDefault(0.0);
                TOP_Kv.initDefault(0.112);
                TOP_Ka.initDefault(0.0);

                BOTTOM_P.initDefault(10.0);
                BOTTOM_I.initDefault(0.0);
                BOTTOM_D.initDefault(0.0);
                BOTTOM_Ks.initDefault(0.0);
                BOTTOM_Kv.initDefault(0.112);
                BOTTOM_Ka.initDefault(0.0);
            default:
                TOP_P.initDefault(0.0);
                TOP_I.initDefault(0.0);
                TOP_D.initDefault(0.0);
                TOP_Ks.initDefault(0.0);
                TOP_Kv.initDefault(0.0);
                TOP_Ka.initDefault(0.0);

                BOTTOM_P.initDefault(0.0);
                BOTTOM_I.initDefault(0.0);
                BOTTOM_D.initDefault(0.0);
                BOTTOM_Ks.initDefault(0.0);
                BOTTOM_Kv.initDefault(0.0);
                BOTTOM_Ka.initDefault(0.0);
        }
    }
}
