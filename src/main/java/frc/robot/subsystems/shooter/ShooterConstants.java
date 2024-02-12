package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.utils.ShootingCSV;
import lib.math.interpolation.InterpolatingDoubleMap;
import lib.webconstants.LoggedTunableNumber;

public class ShooterConstants {
    public static final double GEAR_RATIO_TOP = 1.0;
    public static final double GEAR_RATIO_BOTTOM = 1.0;
    public static final MutableMeasure<Dimensionless> SETPOINT_TOLERANCE_TOP =
            Units.Percent.of(0.05).mutableCopy();
    public static final MutableMeasure<Dimensionless> SETPOINT_TOLERANCE_BOTTOM =
            Units.Percent.of(0.05).mutableCopy();
    public static final MutableMeasure<Mult<Mult<Mass, Distance>, Distance>> MOMENT_OF_INERTIA_TOP =
            Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0008).mutableCopy();
    public static final MutableMeasure<Mult<Mult<Mass, Distance>, Distance>>
            MOMENT_OF_INERTIA_BOTTOM =
                    Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0008).mutableCopy();
    public static final MutableMeasure<Distance> SHOOTER_HEIGHT =
            Units.Meters.of(0.4).mutableCopy(); // TODO: add real value
    public static final double MAX_WARMUP_DISTANCE = 12; // [m] //TODO: add real value

    public static final TalonFXConfiguration topMotorConfiguration = new TalonFXConfiguration();
    public static final TalonFXConfiguration bottomMotorConfiguration = new TalonFXConfiguration();

    public static final double CURRENT_LIMIT_TOP = 40;
    public static final double CURRENT_LIMIT_BOTTOM = 40;

    public static final InvertedValue TOP_INVERSION = InvertedValue.Clockwise_Positive;
    public static final InvertedValue BOTTOM_INVERSION = InvertedValue.CounterClockwise_Positive;

    public static final InterpolatingDoubleMap VELOCITY_BY_DISTANCE =
            ShootingCSV.parse(
                    Filesystem.getDeployDirectory()
                            + "\\distance-to-velocity.csv"); // Velocity | Distance
    public static final InterpolatingDoubleMap FLIGHT_TIME_BY_DISTANCE =
            ShootingCSV.parse(
                    Filesystem.getDeployDirectory()
                            + "\\distance-to-flight-time.csv"); // Flight Time | Distance

    public static final LoggedTunableNumber TOP_kP = new LoggedTunableNumber("Shooter/Top kP");
    public static final LoggedTunableNumber TOP_kI = new LoggedTunableNumber("Shooter/Top kI");
    public static final LoggedTunableNumber TOP_kD = new LoggedTunableNumber("Shooter/Top kD");
    public static final LoggedTunableNumber TOP_kS = new LoggedTunableNumber("Shooter/Top kS");
    public static final LoggedTunableNumber TOP_kV = new LoggedTunableNumber("Shooter/Top kV");
    public static final LoggedTunableNumber TOP_kA = new LoggedTunableNumber("Shooter/Top kA");

    public static final LoggedTunableNumber BOTTOM_kP =
            new LoggedTunableNumber("Shooter/Bottom kP");
    public static final LoggedTunableNumber BOTTOM_kI =
            new LoggedTunableNumber("Shooter/Bottom kI");
    public static final LoggedTunableNumber BOTTOM_kD =
            new LoggedTunableNumber("Shooter/Bottom kD");
    public static final LoggedTunableNumber BOTTOM_kS =
            new LoggedTunableNumber("Shooter/Bottom kS");
    public static final LoggedTunableNumber BOTTOM_kV =
            new LoggedTunableNumber("Shooter/Bottom kV");
    public static final LoggedTunableNumber BOTTOM_kA =
            new LoggedTunableNumber("Shooter/Bottom kA");

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                TOP_kP.initDefault(10.0);
                TOP_kI.initDefault(0.0);
                TOP_kD.initDefault(0.0);
                TOP_kS.initDefault(0.0);
                TOP_kV.initDefault(0.112);
                TOP_kA.initDefault(0.0);
                BOTTOM_kP.initDefault(10.0);
                BOTTOM_kI.initDefault(0.0);
                BOTTOM_kD.initDefault(0.0);
                BOTTOM_kS.initDefault(0.0);
                BOTTOM_kV.initDefault(0.112);
                BOTTOM_kA.initDefault(0.0);
            case SIM:
            case REPLAY:
                TOP_kP.initDefault(10.0);
                TOP_kI.initDefault(0.0);
                TOP_kD.initDefault(0.0);
                TOP_kS.initDefault(0.0);
                TOP_kV.initDefault(0.112);
                TOP_kA.initDefault(0.0);
                BOTTOM_kP.initDefault(10.0);
                BOTTOM_kI.initDefault(0.0);
                BOTTOM_kD.initDefault(0.0);
                BOTTOM_kS.initDefault(0.0);
                BOTTOM_kV.initDefault(0.112);
                BOTTOM_kA.initDefault(0.0);
        }
        topMotorConfiguration
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO_TOP))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(TOP_kP.get())
                                .withKI(TOP_kI.get())
                                .withKD(TOP_kD.get())
                                .withKS(TOP_kS.get())
                                .withKV(TOP_kV.get())
                                .withKA(TOP_kA.get()))
                .withMotorOutput(new MotorOutputConfigs().withInverted(TOP_INVERSION))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT_TOP)
                .withSupplyCurrentLimit(CURRENT_LIMIT_TOP);

        bottomMotorConfiguration
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO_BOTTOM))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(BOTTOM_kP.get())
                                .withKI(BOTTOM_kI.get())
                                .withKD(BOTTOM_kD.get())
                                .withKS(BOTTOM_kS.get())
                                .withKV(BOTTOM_kV.get())
                                .withKA(BOTTOM_kA.get()))
                .withMotorOutput(new MotorOutputConfigs().withInverted(TOP_INVERSION))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT_BOTTOM)
                .withSupplyCurrentLimit(CURRENT_LIMIT_BOTTOM);
    }

    public static MutableMeasure<Velocity<Angle>> STOP_POWER =
            Units.RotationsPerSecond.zero().mutableCopy();
    public static MutableMeasure<Velocity<Angle>> OUTTAKE_POWER =
            Units.RotationsPerSecond.zero().mutableCopy();
}
