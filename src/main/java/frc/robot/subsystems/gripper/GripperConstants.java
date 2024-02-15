package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public class GripperConstants {
    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration();

    public static final Measure<Angle> INTAKE_ANGLE = null;
    public static final Measure<Angle> OUTTAKE_ANGLE = null;
    public static final double INTAKE_POWER = 0;
    public static final double OUTTAKE_POWER = 0;
    public static final Measure<Dimensionless> THRESHOLD = Units.Percent.of(0.02);
    public static final Measure<Distance> GRIPPER_POSITION_X = Units.Meters.of(0);
    public static final Measure<Distance> GRIPPER_POSITION_Y = Units.Meters.of(0);
    public static final Measure<Distance> GRIPPER_POSITION_z = Units.Meters.of(0.6461);
    public static final double ANGLE_MOTOR_GEAR_RATIO = 58.5;
    public static final InvertedValue ANGLE_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    public static final boolean ROLLER_INVERTED_VALUE = true;
    public static final int CURRENT_LIMIT = 40;
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
                KP.initDefault(2);
                KI.initDefault(0);
                KD.initDefault(0);
                KV.initDefault(3.2);
                KA.initDefault(0.01);
                KG.initDefault(0.65);

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
                                .withInverted(InvertedValue.CounterClockwise_Positive))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(40)
                .withSupplyTimeThreshold(1)
                .withSupplyCurrentThreshold(50);
    }
}
