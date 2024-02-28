package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class GyroIOReal implements GyroIO {
    private final AHRS gyro;
    private Rotation2d gyroOffset = new Rotation2d();

    public GyroIOReal() {
        this.gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();
    }

    @Override
    public Rotation2d getYaw() {
        return getRawYaw().minus(gyroOffset);
    }

    @Override
    public Rotation2d getRawYaw() {
        return new Rotation2d(-MathUtil.angleModulus(Math.toRadians(gyro.getAngle())));
    }

    @Override
    public Rotation2d getPitch() {
        return new Rotation2d(gyro.getPitch());
    }

    @Override
    public void resetGyro(Rotation2d angle) {
        gyroOffset = getRawYaw().minus(angle);
    }

    @Override
    public void updateInputs(SwerveDriveInputs inputs) {
        inputs.gyroOffset = gyroOffset;
    }
}
