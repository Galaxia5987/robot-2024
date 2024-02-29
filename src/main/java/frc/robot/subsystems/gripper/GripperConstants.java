package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public class GripperConstants {
    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration();
    public static final double INTAKE_POWER = 0.7; // TODO; replace with actual value
    public static final double OUTTAKE_POWER = -0.5; // TODO: replace with actual value
    public static final double AMP_POWER_NORMAL = 0.7; // TODO: replace with actual value
    public static final double AMP_POWER_REVERSE = -0.4; // TODO: replace with actual value
    public static final double TRAP_POWER = 0; // TODO: replace with actual value
    public static final MutableMeasure<Dimensionless> TOLERANCE =
            Units.Percent.of(0.07).mutableCopy();
    public static final MutableMeasure<Distance> GRIPPER_OUTTAKE_MIN_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // TODO: replace with actual value
    public static final MutableMeasure<Distance> GRIPPER_LENGTH =
            Units.Meters.of(0.534_35).mutableCopy();
    public static final MutableMeasure<Distance> GRIPPER_POSITION_X =
            Units.Meters.of(0).mutableCopy();
    public static final MutableMeasure<Distance> GRIPPER_POSITION_Y =
            Units.Meters.of(0).mutableCopy();
    public static final MutableMeasure<Distance> GRIPPER_POSITION_z =
            Units.Meters.of(0.6461).mutableCopy();
    public static final Measure<Angle> INTAKE_ANGLE = Units.Degrees.of(-85.78);
    public static final Measure<Angle> OUTTAKE_ANGLE = Units.Degrees.of(-80);
    public static final double ANGLE_MOTOR_GEAR_RATIO = 58.5;
    public static final InvertedValue ANGLE_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    public static final boolean ROLLER_INVERTED_VALUE = true;
    public static final int CURRENT_LIMIT = 40;
    public static final MutableMeasure<Angle> WRIST_TRAP_ANGLE =
            Units.Degrees.of(120).mutableCopy(); // TODO: replace with actual value
    public static final LoggedTunableNumber ABSOLUTE_ENCODER_OFFSET =
            new LoggedTunableNumber("Gripper/EncoderOffset");

    public static final LoggedTunableNumber KP = new LoggedTunableNumber("Gripper/kP");
    public static final LoggedTunableNumber KI = new LoggedTunableNumber("Gripper/kI");
    public static final LoggedTunableNumber KD = new LoggedTunableNumber("Gripper/kD");
    public static final LoggedTunableNumber KV = new LoggedTunableNumber("Gripper/kV");
    public static final LoggedTunableNumber KA = new LoggedTunableNumber("Gripper/kA");
    public static final LoggedTunableNumber KG = new LoggedTunableNumber("Gripper/kG");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                KP.initDefault(0);
                KI.initDefault(0);
                KD.initDefault(0);
                KV.initDefault(9);
                KA.initDefault(0.0);
                KG.initDefault(0.4);

                ABSOLUTE_ENCODER_OFFSET.initDefault(-0.682_385 + 83 / 360.0);
                break;
            case SIM:
            case REPLAY:
            default:
                KP.initDefault(24);
                KI.initDefault(0.000_001);
                KD.initDefault(0);
                break;
        }

        MOTOR_CONFIGURATION
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicExpo_kV(KV.get())
                                .withMotionMagicExpo_kA(KA.get())
                                .withMotionMagicAcceleration(3)
                                .withMotionMagicCruiseVelocity(1))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(GripperConstants.KP.get())
                                .withKI(GripperConstants.KI.get())
                                .withKD(GripperConstants.KD.get())
                                .withKV(GripperConstants.KV.get())
                                .withKA(GripperConstants.KA.get())
                                .withKG(GripperConstants.KG.get())
                                .withGravityType(GravityTypeValue.Arm_Cosine))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(40)
                .withSupplyTimeThreshold(1)
                .withSupplyCurrentThreshold(50);
    }
}
