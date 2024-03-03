package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import frc.robot.Constants;
import frc.robot.lib.webconstants.LoggedTunableNumber;

public class IntakeConstants {
    public static final double GEAR_RATIO = 45.62;
    public static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("Intake/Angle/kP");
    public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("Intake/Angle/kI");
    public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("Intake/Angle/kD");
    public static final LoggedTunableNumber ANGLE_KG = new LoggedTunableNumber("Intake/Angle/kG");

    public static final double ANGLE_GEAR_RATIO = 44.44;
    public static final double ANGLE_CURRENT_LIMIT = 25;
    public static final int SPIN_CURRENT_LIMIT = 40;
    public static final int CENTER_CURRENT_LIMIT = 40;
    public static final TalonFXConfiguration ANGLE_CONFIGURATION = new TalonFXConfiguration();

    public static void initConstants() {

        switch (Constants.CURRENT_MODE) {
            case REAL:
                ANGLE_KP.initDefault(0.09);
                ANGLE_KI.initDefault(0.03);
                ANGLE_KD.initDefault(0);
                ANGLE_KG.initDefault(0.3);
                break;
            case SIM:
            case REPLAY:
            default:
                ANGLE_KP.initDefault(10.1 / 360.0);
                ANGLE_KI.initDefault(0);
                ANGLE_KD.initDefault(0);
                break;
        }

        ANGLE_CONFIGURATION
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ANGLE_KP.get())
                                .withKI(ANGLE_KI.get())
                                .withKD(ANGLE_KD.get())
                                .withKG(ANGLE_KG.get())
                                .withGravityType(GravityTypeValue.Elevator_Static))
                .withMotorOutput(
                        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(ANGLE_CURRENT_LIMIT)
                                .withSupplyCurrentLimit(ANGLE_CURRENT_LIMIT))
                .withFeedback(
                        new FeedbackConfigs().withSensorToMechanismRatio(ANGLE_GEAR_RATIO / 360));
    }

    public enum IntakePose {
        UP(Degrees.of(110).mutableCopy()),
        DOWN(Degrees.zero().mutableCopy());
        public final MutableMeasure<Angle> intakePose;

        IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }
    }
}
