package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Utils {
    public static double normalize (double angleRad){
        while(angleRad<0){
            angleRad +=2*Math.PI;
        }
        return angleRad %(2*Math.PI);
    }

    public static Rotation2d normalizeRotations (double angleRotations){
        return Rotation2d.fromRadians(normalize(angleRotations * Constants.toRadiansFromRotation));
    }
}
