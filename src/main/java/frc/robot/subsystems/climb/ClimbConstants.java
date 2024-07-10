package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;

public class ClimbConstants { // TODO: check real values
    public static final TalonFXConfiguration MAIN_MOTOR_CONFIGURATION = new TalonFXConfiguration();
    public static final TalonFXConfiguration AUX_MOTOR_CONFIGURATION = new TalonFXConfiguration();

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs();
    public static final Slot0Configs PID_CONFIGS = new Slot0Configs();

    public static final double GEAR_RATIO = 12.0;

    public static final MutableMeasure<Angle> OPEN_POSITION = Units.Degrees.of(40).mutableCopy();
    public static final MutableMeasure<Angle> LOCKED_POSITION = Units.Degrees.of(140).mutableCopy();

    public static void initConstants() {
        CURRENT_LIMITS_CONFIGS
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(40);

        PID_CONFIGS
                .withKP(0.1).withKI(0).withKD(0.005).withKG(0);

        MAIN_MOTOR_CONFIGURATION
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                .withSlot0(PID_CONFIGS);


        AUX_MOTOR_CONFIGURATION.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));
    }
}
