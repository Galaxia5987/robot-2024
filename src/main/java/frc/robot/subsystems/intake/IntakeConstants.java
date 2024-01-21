package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;

public class IntakeConstants {
    public static final double ANGLE_RATIO = 0;
    public static MutableMeasure<Angle> DOWN_ANGLE = MutableMeasure.zero(Radians);
    public static MutableMeasure<Angle> UP_ANGLE = MutableMeasure.zero(Radians);

    public enum IntakePose {
        UP(IntakeConstants.UP_ANGLE),
        DOWN(IntakeConstants.DOWN_ANGLE);
         final MutableMeasure<Angle> intakePose;

        IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }
    }
}
