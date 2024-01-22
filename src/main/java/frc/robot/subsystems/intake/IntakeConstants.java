package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;

public class IntakeConstants {
    public static final double GEAR_RATIO = 0;
    public static final double ANGLE_KP = 0;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;

    public enum IntakePose {
        UP(MutableMeasure.zero(Degrees)),
        DOWN(MutableMeasure.zero(Degrees));
        final MutableMeasure<Angle> intakePose;

        IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }
    }
}
