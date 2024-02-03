package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import frc.robot.Constants;
import lib.webconstants.LoggedTunableNumber;

public class IntakeConstants {
    public static final double GEAR_RATIO = 45.62;
    public static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("Intake/kP");
    public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("Intake/kI");
    public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("Intake/kD");
    public static final double ANGLE_GEAR_RATIO = 44.44;


    public enum IntakePose {
        UP(MutableMeasure.zero(Degrees)),
        DOWN(MutableMeasure.zero(Degrees));
        final MutableMeasure<Angle> intakePose;

        IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }
    }

    public void initConstants() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                ANGLE_KP.initDefault(10.0 / 360.0);
                ANGLE_KI.initDefault(0);
                ANGLE_KD.initDefault(0);
                break;
            case SIM:
            case REPLAY:
            default:
                ANGLE_KP.initDefault(10.0 / 360.0);
                ANGLE_KI.initDefault(0);
                ANGLE_KD.initDefault(0);
                break;
        }
    }
}
