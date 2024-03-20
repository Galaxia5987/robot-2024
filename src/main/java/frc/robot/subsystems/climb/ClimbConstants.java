package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;

public class ClimbConstants { // TODO: check real values
    public static final TalonFXConfiguration MAIN_MOTOR_CONFIGURATION = new TalonFXConfiguration();
    public static final TalonFXConfiguration AUX_MOTOR_CONFIGURATION = new TalonFXConfiguration();

    public static final double GEAR_RATIO = 12.0;

    public static final MutableMeasure<Angle> OPEN_POSITION = Units.Degrees.of(35).mutableCopy();
    public static final MutableMeasure<Angle> LOCKED_POSITION = Units.Degrees.of(140).mutableCopy();

    public static void initConstants() {
        MAIN_MOTOR_CONFIGURATION
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .CurrentLimits
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimitEnable(false);

        AUX_MOTOR_CONFIGURATION.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));
    }
}
