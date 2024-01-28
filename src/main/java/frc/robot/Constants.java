package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {

    public static final int CONFIG_TIMEOUT = 100; // [ms]

    public static final Mode CURRENT_MODE = Mode.SIM;

    public static final Transform3d BACK_LEFT_CAMERA_POSE =
            new Transform3d(
                    -0.293,
                    0.293,
                    0.2,
                    new Rotation3d(0, Math.toRadians(31.92), Math.toRadians(180)));
    public static final Transform3d BACK_RIGHT_CAMERA_POSE =
            new Transform3d(
                    -0.293,
                    -0.293,
                    0.2,
                    new Rotation3d(0, Math.toRadians(31.92), Math.toRadians(-100)));
    public static final Transform3d FRONT_LEFT_CAMERA_POSE =
            new Transform3d(0, 0.293, 0.55, new Rotation3d(0, Math.toRadians(10.0), 0));
    public static final Transform3d FRONT_RIGHT_CAMERA_POSE =
            new Transform3d(0, -0.293, 0.55, new Rotation3d(0, Math.toRadians(25.0), 0));

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
