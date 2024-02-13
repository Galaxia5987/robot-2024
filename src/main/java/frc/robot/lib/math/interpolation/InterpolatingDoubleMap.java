package frc.robot.lib.math.interpolation;

import lib.math.interpolation.InterpolatingDouble;
import lib.math.interpolation.InterpolatingTreeMap;

public class InterpolatingDoubleMap
        extends InterpolatingTreeMap<lib.math.interpolation.InterpolatingDouble, lib.math.interpolation.InterpolatingDouble> {

    public InterpolatingDoubleMap(int maximumSize) {
        super(maximumSize);
    }

    public InterpolatingDoubleMap() {}

    public lib.math.interpolation.InterpolatingDouble put(double a, double b) {
        return super.put(new lib.math.interpolation.InterpolatingDouble(a), new InterpolatingDouble(b));
    }
}
