package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class HoodConstants {
    public static Translation2d MECHANISM_2D_POSE = new Translation2d(1, 1);
    public static double HOOD_LENGTH = 0.4; // m
    public static Measure<Dimensionless> POSITION_TOLERANCE = Units.Percent.of(5);
}
