package frc.robot.subsystems.hood;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class HoodKinematics {

    public static final Measure<Distance> AXIS_HEIGHT = Units.Meters.of(0.2049);
    public static final Measure<Distance> SPEAKER_HEIGHT = Units.Meters.of(1.98);
    public static final Measure<Angle> NOTE_OUT_OFFSET = Units.Degrees.of(47.5);
    public static final Measure<Distance> AXIS_TO_ROBOT_MIDDLE = Units.Meters.of(0.29);

    public static Measure<Angle> getAngleFromDistance(Measure<Distance> distanceFromSpeaker) {
        Measure<Distance> deltaHeight = SPEAKER_HEIGHT.minus(AXIS_HEIGHT);
        return Units.Radians.of(
                        Math.atan2(
                                deltaHeight.in(Units.Meters),
                                distanceFromSpeaker.minus(AXIS_TO_ROBOT_MIDDLE).in(Units.Meters)))
                .plus(NOTE_OUT_OFFSET);
    }
}
