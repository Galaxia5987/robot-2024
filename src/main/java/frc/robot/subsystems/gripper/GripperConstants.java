package frc.robot.subsystems.gripper;

import edu.wpi.first.units.*;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class GripperConstants {
    public static final Measure<Angle> INTAKE_ANGLE =
            Units.Rotations.of(0); // TODO: replace with actual value
    public static final Measure<Angle> OUTTAKE_ANGLE =
            Units.Rotations.of(0); // TODO: replace with actual value
    public static final double INTAKE_POWER = 0; // TODO; replace with actual value
    public static final double OUTTAKE_POWER = 0; // TODO: replace with actual value
    public static final double AMP_POWER_NORMAL = 0;
    public static final double AMP_POWER_REVERSE = 0;
    public static final double TRAP_POWER = 0;
    public static final Measure<Dimensionless> THRESHOLD = Units.Percent.of(0.02);
    public static final Measure<Distance> GRIPPER_OUTTAKE_MIN_HEIGHT =
            Units.Meters.of(0); // TODO: replace with actual value
    public static final Measure<Distance> GRIPPER_LENGTH = Units.Meters.of(0.534_35);
    public static final Measure<Distance> GRIPPER_POSITION_X = Units.Meters.of(0);
    public static final Measure<Distance> GRIPPER_POSITION_Y = Units.Meters.of(0);
    public static final Measure<Distance> GRIPPER_POSITION_z = Units.Meters.of(0.6461);
    public static final double ANGLE_MOTOR_GEAR_RATIO = 58.5;

    public static final Measure<Angle> WRIST_FOLD_POSITION =
            Units.Rotations.of(0.5); // TODO: replace with actual value
    public static final Measure<Angle> WRIST_FORWARD_AMP_POSE =
            Units.Rotations.of(0.3); // TODO: replace with actual value;
    public static final Measure<Angle> WRIST_BACKWARDS_AMP =
            Units.Rotations.of(0.6); // TODO: replace with actual value;
    public static final Measure<Angle> WRIST_TRAP_ANGLE = Units.Rotations.of(0.0);
    public static final Measure<Angle> WRIST_TOLERANCE = Units.Rotations.of(0.05);

    public static final LoggedTunableNumber KP = new LoggedTunableNumber("Gripper/kP");
    public static final LoggedTunableNumber KI = new LoggedTunableNumber("Gripper/kI");
    public static final LoggedTunableNumber KD = new LoggedTunableNumber("Gripper/kD");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                KP.initDefault(0);
                KI.initDefault(0);
                KD.initDefault(0);
                break;
            case SIM:
            case REPLAY:
            default:
                KP.initDefault(24);
                KI.initDefault(0.000_001);
                KD.initDefault(0);
                break;
        }
    }
}
