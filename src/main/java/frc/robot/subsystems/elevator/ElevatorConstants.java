package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public class ElevatorConstants { // TODO: check real values
    public static final TalonFXConfiguration MAIN_MOTOR_CONFIGURATION = new TalonFXConfiguration();
    public static final TalonFXConfiguration AUX_MOTOR_CONFIGURATION = new TalonFXConfiguration();

    public static final double MECHANISM_WIDTH = 0.8; // [m]
    public static final double MECHANISM_HEIGHT = 2; // [m]
    public static final double GEAR_RATIO = 12.0;
    public static final double DRUM_RADIUS = 0.0125; // [m]
    public static final double CIRCUMFERENCE = DRUM_RADIUS * (2 * Math.PI);

    public static final MutableMeasure<Angle> OPEN_POSITION = Units.Degrees.of(0).mutableCopy();
    public static final MutableMeasure<Angle> LOCKED_POSITION = Units.Degrees.of(90).mutableCopy();
    public static final MutableMeasure<Dimensionless> HEIGHT_TOLERANCE =
            Units.Percent.of(1).mutableCopy();
    public static final MutableMeasure<Dimensionless> STOPPER_TOLERANCE =
            Units.Percent.of(2).mutableCopy();

    public static final MutableMeasure<Mass> HOOKS_MASS =
            Units.Kilograms.of(1).mutableCopy(); // TODO: Calibrate real value
    public static final MutableMeasure<Mass> ELEVATOR_MASS =
            Units.Kilograms.of(5).mutableCopy(); // TODO: Calibrate real value

    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCELERATION = 3;

    public static final Measure<Distance> GRIPPER_TO_HOOKS =
            Units.Meters.of(0.335); // TODO: Calibrate real value

    public static final TrapezoidProfile.Constraints TRAPEZOID_PROFILE =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);

    public static final LoggedTunableNumber KP = new LoggedTunableNumber("kp");
    public static final LoggedTunableNumber KI = new LoggedTunableNumber("ki");
    public static final LoggedTunableNumber KD = new LoggedTunableNumber("kd");
    public static final LoggedTunableNumber KV = new LoggedTunableNumber("kv");
    public static final LoggedTunableNumber KA = new LoggedTunableNumber("ka");
    public static final LoggedTunableNumber KS_FIRST_STAGE =
            new LoggedTunableNumber("ks_first_stage_up");
    public static final LoggedTunableNumber KS_SECOND_STAGE =
            new LoggedTunableNumber("ks_second_stage");
    public static final LoggedTunableNumber KG_FIRST_STAGE =
            new LoggedTunableNumber("kg_first_stage");
    public static final LoggedTunableNumber KG_SECOND_STAGE =
            new LoggedTunableNumber("kg_second_stage");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                KP.initDefault(0.0);
                KI.initDefault(0.0);
                KD.initDefault(0.0);
                KV.initDefault(0.0);
                KA.initDefault(0.0);
                KS_FIRST_STAGE.initDefault(0.62);
                KS_SECOND_STAGE.initDefault(0.22);
                KG_FIRST_STAGE.initDefault(-0.60);
                KG_SECOND_STAGE.initDefault(-0.15);
            case SIM:
            case REPLAY:
                KP.initDefault(43.0);
                KI.initDefault(0.0003);
                KD.initDefault(0.05);
        }

        MAIN_MOTOR_CONFIGURATION
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(MAX_ACCELERATION)
                                .withMotionMagicCruiseVelocity(MAX_VELOCITY))
                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(GEAR_RATIO / CIRCUMFERENCE))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ElevatorConstants.KP.get())
                                .withKI(ElevatorConstants.KI.get())
                                .withKD(ElevatorConstants.KD.get())
                                .withKV(ElevatorConstants.KV.get())
                                .withKA(ElevatorConstants.KA.get())
                                .withGravityType(GravityTypeValue.Elevator_Static))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(80)
                .withSupplyCurrentLimit(50);

        AUX_MOTOR_CONFIGURATION
                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(GEAR_RATIO / CIRCUMFERENCE))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake));
    }
}
