package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.lib.ShootingCSV;
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public class HoodConstants {
    public static Translation2d MECHANISM_2D_POSE = new Translation2d(1, 1);
    public static Measure<Distance> HOOD_LENGTH = Units.Meters.of(0.4);
    public static Measure<Dimensionless> POSITION_TOLERANCE = Units.Percent.of(0.5);
    public static Measure<Velocity<Angle>> MAX_VELOCITY = Units.RotationsPerSecond.of(1);
    public static final MutableMeasure<Angle> AMP_ANGLE = Units.Degrees.of(118).mutableCopy();
    public static Measure<Velocity<Velocity<Angle>>> MAX_ACCELERATION =
            Units.RotationsPerSecond.per(Units.Second).of(4);
    public static final double GEAR_RATIO = 3.0 * (36.0 / 18.0) * (158.0 / 18.0);
    public static final Measure<Mult<Mult<Mass, Distance>, Distance>> MOMENT_OF_INERTIA =
            Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0003);
    public static final Translation3d ROOT_POSITION = new Translation3d(-0.27, 0, 0.225);
    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration();
    public static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    public static final double CURRENT_LIMIT = 40;
    public static final Measure<Angle> SIMULATION_OFFSET = Units.Degrees.of(-54);
    public static final LoggedTunableNumber ABSOLUTE_ENCODER_OFFSET =
            new LoggedTunableNumber("Hood/EncoderOffset");

    public static final int ENCODER_TICKS_PER_REVOLUTION = 4096;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI");
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV");
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG");

    public static final MutableMeasure<Angle> FOLDED_ANGLE = Units.Degrees.of(90).mutableCopy();

    public static final InterpolatingDoubleMap ANGLE_BY_DISTANCE =
            ShootingCSV.parse(Filesystem.getDeployDirectory() + "/shootdata/distance-to-angle.csv");

    public static final double TORQUE_TO_CURRENT =
            DCMotor.getFalcon500(1).KtNMPerAmp / HoodConstants.GEAR_RATIO;
    public static final Measure<Mass> MASS = Units.Kilograms.of(7.5);
    public static final Measure<Distance> CM_RADIUS = Units.Millimeters.of(110);
    public static final Measure<Distance> AXIS_DISTANCE_TO_CENTER = Units.Millimeters.of(270);

    public static void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                kP.initDefault(900);
                kI.initDefault(0);
                kD.initDefault(150);
                kS.initDefault(0.5);
                kV.initDefault(0);
                kA.initDefault(0.0);
                kG.initDefault(9.0);
                ABSOLUTE_ENCODER_OFFSET.initDefault((76.9 - 33.48) / 360.0);
            case SIM:
            case REPLAY:
                kP.initDefault(10);
                kI.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
        }
        MOTOR_CONFIGURATION
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(
                                        MAX_VELOCITY.in(Units.RotationsPerSecond))
                                .withMotionMagicJerk(16))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(HoodConstants.kP.get())
                                .withKI(HoodConstants.kI.get())
                                .withKD(HoodConstants.kD.get())
                                .withKS(HoodConstants.kS.get())
                                .withKV(HoodConstants.kV.get())
                                .withKA(HoodConstants.kA.get())
                                .withKG(HoodConstants.kG.get())
                                .withGravityType(GravityTypeValue.Arm_Cosine))
                .withMotorOutput(new MotorOutputConfigs().withInverted(INVERTED_VALUE))
                .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimit(CURRENT_LIMIT);
    }
}
