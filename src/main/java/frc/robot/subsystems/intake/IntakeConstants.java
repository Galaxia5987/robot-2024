package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import lib.webconstants.LoggedTunableNumber;

public class IntakeConstants {
    public static final double GEAR_RATIO = 0;
    public static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("IntakeSim_kP",0);
    public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("IntakeSim_kI",0);
    public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("IntakeSim_kD",0);
    public enum IntakePose {
        UP(MutableMeasure.zero(Degrees)),
        DOWN(MutableMeasure.zero(Degrees));
        final MutableMeasure<Angle> intakePose;


        IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }
    }
}
