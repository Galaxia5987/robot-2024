package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.subsystems.hood.HoodConstants;
import lib.webconstants.LoggedTunableNumber;

public class ElevatorConstants { // TODO: check real values
    public static final TalonFXConfiguration MAIN_MOTOR_CONFIGURATION = new TalonFXConfiguration();
    public static final TalonFXConfiguration AUX_MOTOR_CONFIGURATION = new TalonFXConfiguration();

    public static final double MECHANISM_WIDTH = 0.8; // [m]
    public static final double MECHANISM_HEIGHT = 2; // [m]
    public static final double GEAR_RATIO = 12.0;
    public static final double DRUM_RADIUS = 0.02; // [m]


    public static final MutableMeasure<Distance> STARTING_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Distance> MIN_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Distance> MAX_HEIGHT =
            Units.Meters.of(0).mutableCopy(); // [m]

    public static final MutableMeasure<Angle> SERVO_OPEN = Units.Degrees.of(0).mutableCopy(); // [m]
    public static final MutableMeasure<Angle> SERVO_CLOSE =
            Units.Degrees.of(0).mutableCopy(); // [m]

    public static final InvertedValue MAIN_INVERT = InvertedValue.Clockwise_Positive;
    public static final InvertedValue AUX_INVERT = InvertedValue.Clockwise_Positive;
    public static final double CURRENT_LIMIT = 40;

    public static final double MAX_VELOCITY = 3;
    public static final double MAX_ACCELERATION = 7;

    public static final Measure<Distance> GRIPPER_HEIGHT = Units.Meters.of(0.3); // [m]

    public static final TrapezoidProfile.Constraints TRAPEZOID_PROFILE =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);

    public static final LoggedTunableNumber KP = new LoggedTunableNumber("kp");
    public static final LoggedTunableNumber KI = new LoggedTunableNumber("ki");
    public static final LoggedTunableNumber KD = new LoggedTunableNumber("kd");
    public static final LoggedTunableNumber KV = new LoggedTunableNumber("kd");
    public static final LoggedTunableNumber KA = new LoggedTunableNumber("kd");
    public static final LoggedTunableNumber KG = new LoggedTunableNumber("kg");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                KP.initDefault(0.8);
                KI.initDefault(0.0);
                KD.initDefault(0.0);
                KV.initDefault(0.0);
                KA.initDefault(0.0);
                KG.initDefault(0.0);
            case SIM:
            case REPLAY:
                KP.initDefault(43.0);
                KI.initDefault(0.0003);
                KD.initDefault(0.05);
        }

        MAIN_MOTOR_CONFIGURATION.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ElevatorConstants.KP.get())
                                .withKI(ElevatorConstants.KI.get())
                                .withKD(ElevatorConstants.KD.get())
                                .withKG(ElevatorConstants.KG.get()))
                .withMotorOutput(new MotorOutputConfigs().withInverted(MAIN_INVERT))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimit(CURRENT_LIMIT);

        AUX_MOTOR_CONFIGURATION.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO))
                .withMotorOutput(new MotorOutputConfigs().withInverted(AUX_INVERT));
    }
}
