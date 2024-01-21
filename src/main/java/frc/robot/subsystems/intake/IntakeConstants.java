package frc.robot.subsystems.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.Radians;

public class IntakeConstants {
    public static final double ANGLE_RATIO = 0;
    public static final MutableMeasure<Angle> DOWN_ANGLE = MutableMeasure.zero(Radians);
    public static final MutableMeasure<Angle> UP_ANGLE = MutableMeasure.zero(Radians);
    public enum IntakePose {
        UP(IntakeConstants.UP_ANGLE),
        DOWN(IntakeConstants.DOWN_ANGLE);
        MutableMeasure<Angle> intakePose;
        private IntakePose(MutableMeasure<Angle> intakePose){
            this.intakePose = intakePose;
        }
    }
}
