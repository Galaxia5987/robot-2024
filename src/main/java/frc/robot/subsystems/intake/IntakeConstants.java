package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
<<<<<<< HEAD
=======

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
>>>>>>> b38babce7c2115f5fefb513ee61988edc8bcbf20

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;

public class IntakeConstants {
    public static final double ANGLE_RATIO = 0;
<<<<<<< HEAD
    public static MutableMeasure<Angle> DOWN_ANGLE = MutableMeasure.zero(Radians);
    public static MutableMeasure<Angle> UP_ANGLE = MutableMeasure.zero(Radians);
=======
    public static final MutableMeasure<Angle> DOWN_ANGLE = MutableMeasure.zero(Radians);
    public static final MutableMeasure<Angle> UP_ANGLE = MutableMeasure.zero(Radians);
>>>>>>> b38babce7c2115f5fefb513ee61988edc8bcbf20

    public enum IntakePose {
        UP(IntakeConstants.UP_ANGLE),
        DOWN(IntakeConstants.DOWN_ANGLE);
        MutableMeasure<Angle> intakePose;

        private IntakePose(MutableMeasure<Angle> intakePose) {
            this.intakePose = intakePose;
        }
    }
}
