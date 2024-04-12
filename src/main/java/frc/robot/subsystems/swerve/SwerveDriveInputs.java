package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveDriveInputs {
    // x, y, omega
    public ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    public double acceleration = 0;

    public Rotation2d rawYaw = new Rotation2d();
    public Rotation2d yaw = new Rotation2d();
    public Rotation2d gyroOffset = new Rotation2d();
}
