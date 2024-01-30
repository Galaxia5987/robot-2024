package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class HoodConstants {
    public static Translation2d MECHANISM_2D_POSE = new Translation2d(1, 1);
    public static Measure<Distance> HOOD_LENGTH = Units.Meters.of(0.4);
    public static Measure<Dimensionless> POSITION_TOLERANCE = Units.Percent.of(5);
    public static double MAX_VELOCITY = 0; //RPS
    public static double MAX_ACCELERATION = 0; //RPS squared
    public static final double GEAR_RATIO = 1.0;
    public static final double MOMENT_OF_INERTIA = 0.08;
    public static final double HOOD_X = -0.27;
    public static final double HOOD_Y = 0.2385;
    public static final Translation2d ROOT_POSITION = new Translation2d(HOOD_X, HOOD_Y);

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI");
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV");
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");

    public static void initialize(Constants.Mode mode) {
        switch (mode) {
            case REAL:
                kP.initDefault(5.0);
                kI.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
            case SIM:
            case REPLAY:
                kP.initDefault(5.0);
                kI.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
        }
    }
}
