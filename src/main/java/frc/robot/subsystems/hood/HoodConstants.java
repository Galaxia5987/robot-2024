package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class HoodConstants {
    public static Translation2d MECHANISM_2D_POSE = new Translation2d(1, 1);
    public static Measure<Distance> HOOD_LENGTH = Units.Meters.of(0.4);
    public static Measure<Dimensionless> POSITION_TOLERANCE = Units.Percent.of(5);
    public static Measure<Velocity<Angle>> MAX_VELOCITY = Units.RotationsPerSecond.of(1);
    public static Measure<Velocity<Velocity<Angle>>> MAX_ACCELERATION =
            Units.RotationsPerSecond.per(Units.Second).of(4);
    public static final double GEAR_RATIO = 1.0;
    public static final Measure<Mult<Mult<Mass, Distance>, Distance>> MOMENT_OF_INERTIA =
            Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0003);
    public static final Translation3d ROOT_POSITION = new Translation3d(-0.27, 0.2385, 0.0);
    public static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    public static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    public static final double CURRENT_LIMIT = 40;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI");
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV");
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG");

    public static void initialize(Constants.Mode mode) {
        switch (mode) {
            case REAL:
                kP.initDefault(1);
                kI.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
                kG.initDefault(0.0);
            case SIM:
            case REPLAY:
                kP.initDefault(1);
                kI.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
        }
        motorConfiguration
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(HoodConstants.kP.get())
                                .withKI(HoodConstants.kI.get())
                                .withKD(HoodConstants.kD.get())
                                .withKS(HoodConstants.kS.get())
                                .withKV(HoodConstants.kV.get())
                                .withKA(HoodConstants.kA.get())
                                .withKG(HoodConstants.kG.get()))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimit(CURRENT_LIMIT);
    }
}
