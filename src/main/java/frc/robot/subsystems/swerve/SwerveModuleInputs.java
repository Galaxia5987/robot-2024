package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveModuleInputs {
    public double driveMotorVelocity = 0;
    public double driveMotorVelocitySetpoint = 0;
    public double driveMotorSupplyCurrent = 0;
    public double driveMotorStatorCurrent = 0;
    public double driveMotorPosition = 0;
    public double driveMotorAppliedVoltage = 0;

    public Rotation2d angle = new Rotation2d();
    public Rotation2d angleSetpoint = new Rotation2d();
    public double absolutePosition = 0;
    public double angleMotorVelocity = 0;
    public double angleMotorSupplyCurrent = 0;
    public double angleMotorStatorCurrent = 0;
    public double angleMotorPosition = 0;
    public double angleMotorAppliedVoltage = 0;

    public double moduleDistance = 0;
    public SwerveModuleState moduleState = new SwerveModuleState();
}
