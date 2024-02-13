package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.motors.TalonFXSim;
import frc.robot.lib.units.Units;

public class ModuleIOSim implements ModuleIO {
    private final TalonFXSim driveMotor;
    private final TalonFXSim angleMotor;
    private final PIDController velocityController;
    private final PIDController angleController;
    private VelocityVoltage driveControlRequest = new VelocityVoltage(0).withEnableFOC(true);
    private PositionVoltage angleControlRequest = new PositionVoltage(0).withEnableFOC(true);
    private double currentVelocity = 0;
    private double velocitySetpoint = 0;
    private Rotation2d currentAngle = new Rotation2d();
    private Rotation2d angleSetpoint = new Rotation2d();

    public ModuleIOSim() {
        driveMotor =
                new TalonFXSim(
                        1,
                        1 / SwerveConstants.DRIVE_REDUCTION,
                        SwerveConstants.DRIVE_MOTOR_MOMENT_OF_INERTIA,
                        1 / SwerveConstants.DRIVE_REDUCTION);

        angleMotor =
                new TalonFXSim(
                        1,
                        1 / SwerveConstants.ANGLE_REDUCTION,
                        SwerveConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA,
                        1 / SwerveConstants.ANGLE_REDUCTION);

        velocityController =
                new PIDController(
                        SwerveConstants.DRIVE_KP.get(),
                        SwerveConstants.DRIVE_KI.get(),
                        SwerveConstants.DRIVE_KD.get(),
                        0.02);
        angleController =
                new PIDController(
                        SwerveConstants.ANGLE_KP.get(),
                        SwerveConstants.ANGLE_KI.get(),
                        SwerveConstants.ANGLE_KD.get(),
                        0.02);

        driveMotor.setController(velocityController);
        angleMotor.setController(angleController);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveMotor.update(Timer.getFPGATimestamp());
        angleMotor.update(Timer.getFPGATimestamp());

        inputs.driveMotorAppliedVoltage = driveMotor.getAppliedVoltage();
        inputs.driveMotorVelocity =
                Units.rpsToMetersPerSecond(
                        driveMotor.getVelocity(), SwerveConstants.WHEEL_DIAMETER / 2);
        currentVelocity = inputs.driveMotorVelocity;
        inputs.driveMotorVelocitySetpoint = velocitySetpoint;

        inputs.angleMotorAppliedVoltage = angleMotor.getAppliedVoltage();
        inputs.angleMotorVelocity = angleMotor.getVelocity();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angle = Rotation2d.fromRotations(angleMotor.getPosition());
        currentAngle = inputs.angle;

        inputs.moduleDistance =
                Units.rpsToMetersPerSecond(
                        driveMotor.getPosition(), SwerveConstants.WHEEL_DIAMETER / 2);
        inputs.moduleState = getModuleState();

        inputs.highFreqDistances = new double[] {inputs.moduleDistance};
        inputs.highFreqAngles = new double[] {inputs.angle.getRadians()};
        inputs.highFreqTimestamps = new double[] {Timer.getFPGATimestamp()};

        if (hasPIDChanged(SwerveConstants.PID_VALUES)) updatePID();
    }

    @Override
    public void updatePID() {
        velocityController.setPID(
                SwerveConstants.DRIVE_KP.get(),
                SwerveConstants.DRIVE_KI.get(),
                SwerveConstants.DRIVE_KD.get());
        angleController.setPID(
                SwerveConstants.ANGLE_KP.get(),
                SwerveConstants.ANGLE_KI.get(),
                SwerveConstants.ANGLE_KD.get());
    }

    @Override
    public Rotation2d getAngle() {
        return currentAngle;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleSetpoint = angle;
        angleMotor.setControl(angleControlRequest.withPosition(angle.getRotations()));
    }

    @Override
    public double getVelocity() {
        return currentVelocity;
    }

    @Override
    public void setVelocity(double velocity) {
        velocitySetpoint = velocity;
        driveControlRequest.withVelocity(
                Units.metersToRotations(velocity, SwerveConstants.WHEEL_DIAMETER / 2));
        driveMotor.setControl(driveControlRequest);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                Units.rpsToMetersPerSecond(
                        driveMotor.getPosition(), SwerveConstants.WHEEL_DIAMETER / 2),
                currentAngle);
    }

    @Override
    public void stop() {
        driveControlRequest.withVelocity(0);
        driveMotor.setControl(driveControlRequest);

        angleControlRequest.withVelocity(0);
        angleMotor.setControl(angleControlRequest);
    }

    @Override
    public Command checkModule() {
        return Commands.run(
                () -> {
                    driveControlRequest.withVelocity(0.8 * SwerveConstants.MAX_X_Y_VELOCITY);
                    driveMotor.setControl(driveControlRequest);
                    angleControlRequest.withVelocity(0.2 * SwerveConstants.MAX_X_Y_VELOCITY);
                    angleMotor.setControl(angleControlRequest);
                });
    }
}
