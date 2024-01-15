package frc.robot.subsystems.example;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class ExampleSubsystemConstants {

    public static final double CURRENT_LIMIT = 40; // [amp]

    public static final LoggedTunableNumber POSITION_P =
            new LoggedTunableNumber("Example/PositionP");
    public static final LoggedTunableNumber POSITION_I =
            new LoggedTunableNumber("Example/PositionI");
    public static final LoggedTunableNumber POSITION_D =
            new LoggedTunableNumber("Example/PositionD");
    public static final LoggedTunableNumber POSITION_V =
            new LoggedTunableNumber("Example/PositionV");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                POSITION_P.initDefault(0.1);
                POSITION_I.initDefault(0.0);
                POSITION_D.initDefault(0.0);
                POSITION_V.initDefault(0.0);
            case SIM:
            case REPLAY:
                POSITION_P.initDefault(0.2);
                POSITION_I.initDefault(0.0);
                POSITION_D.initDefault(0.0);
                POSITION_V.initDefault(0.0);
        }
    }

    public static final double GEAR_RATIO = (32.0 / 15.0) * (20.0 / 15.0);
    public static final double MOMENT_OF_INERTIA = 0.1;

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

    public static final int MOTOR_PORT = 1;

    public static final double LENGTH = 0.5;
}
