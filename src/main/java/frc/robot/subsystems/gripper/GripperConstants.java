package frc.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;

public class GripperConstants {
    public static final MutableMeasure<Angle> INTAKE_ANGLE = null;
    public static final MutableMeasure<Angle> OUTTAKE_ANGLE = null;
    public static final double INTAKE_POWER = 0;
    public static final double OUTTAKE_POWER = 0;
    public static final MutableMeasure<Dimensionless> THRESHOLD =
            Units.Percent.of(0.02).mutableCopy();
    public static final Translation3d GRIPPER_POSITION = new Translation3d(0, 0, 0.6461);
}
