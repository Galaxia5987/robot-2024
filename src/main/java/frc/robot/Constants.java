package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.AutoLogOutput;

public class Constants {

    public static final double toRadiansFromRotation = 6.283_185;

    public static final int CONFIG_TIMEOUT = 100; // [ms]

    public static final double LOOP_TIME = 0.02;
    public static final double AUTO_VISION_MEASUREMENT_MULTIPLIER = 0.5;
    public static final double AUTO_START_VISION_MEASUREMENT_MULTIPLIER = 1_000_000_000;
    public static final double TELEOP_VISION_MEASUREMENT_MULTIPLIER = 0.05;
    public static final Transform3d SPEAKER_RIGHT_CAMERA_POSE =
            new Transform3d(
                    -0.065_833,
                    -0.040_05,
                    0.608_178,
                    new Rotation3d(0, -Math.toRadians(25), Math.toRadians(180)));
    public static final Transform3d SPEAKER_LEFT_CAMERA_POSE =
            new Transform3d(
                    -0.065_833,
                    0.039_95,
                    0.608_178,
                    new Rotation3d(0, -Math.toRadians(25), Math.toRadians(180)));
    public static final Transform3d INTAKE_APRILTAG_CAMERA_POSE =
            new Transform3d(
                    -0.012_852, -0.0005, 0.635_282, new Rotation3d(0, -Math.toRadians(25), 0));
    public static final Transform3d DRIVER_CAMERA_POSE =
            new Transform3d(0.0, 0.0, 0.53, new Rotation3d(0, Math.toRadians(20), 0));
    public static final Measure<Voltage> NOMINAL_VOLTAGE = Units.Volts.of(12);
    public static final Measure<Distance> ROBOT_LENGTH = Units.Meters.of(0.584);
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = Units.MetersPerSecond.of(2.0);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
            Units.MetersPerSecondPerSecond.of(1);
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
            Units.RotationsPerSecond.of(
                    MAX_VELOCITY.in(Units.MetersPerSecond)
                            / (ROBOT_LENGTH.in(Units.Meters) / Math.sqrt(2)));
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION =
            Units.RotationsPerSecond.per(Units.Second)
                    .of(
                            MAX_ACCELERATION.in(Units.MetersPerSecondPerSecond)
                                    / (ROBOT_LENGTH.in(Units.Meters) / Math.sqrt(2)));
    public static final PathConstraints AUTO_CONSTRAINTS =
            new PathConstraints(
                    MAX_VELOCITY.in(Units.MetersPerSecond),
                    MAX_ACCELERATION.in(Units.MetersPerSecondPerSecond),
                    MAX_ANGULAR_VELOCITY.in(Units.RotationsPerSecond),
                    MAX_ANGULAR_ACCELERATION.in(Units.RotationsPerSecond.per(Units.Second)));
    public static final double[] SWERVE_OFFSETS = {
        0.792_610_669_815_266_7,
        0.780_174_219_504_355_5,
        0.522_181_563_054_539_1,
        0.574_873_139_371_828_5
    };

    public static Mode CURRENT_MODE = Mode.REAL;

    @AutoLogOutput
    static double VISION_MEASUREMENT_MULTIPLIER = AUTO_START_VISION_MEASUREMENT_MULTIPLIER;

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public static final Translation2d SPEAKER_POSE_BLUE = new Translation2d(0, 5.547_944_2);
    public static final Translation2d SPEAKER_POSE_RED =
            GeometryUtil.flipFieldPosition(SPEAKER_POSE_BLUE);

    public enum State {
        SHOOT,
        AMP,
        CLIMB
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
