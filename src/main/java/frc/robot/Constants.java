package frc.robot;

public class Constants {

    public static final int CONFIG_TIMEOUT = 100; // [ms]

    public static final Mode CURRENT_MODE = Mode.SIM;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
