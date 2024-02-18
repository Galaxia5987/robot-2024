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
    private final SwerveModuleInputsAutoLogged inputs;

    public ModuleIOSim(SwerveModuleInputsAutoLogged inputs) {
        this.inputs = inputs;

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
    public void updateInputs() {
        driveMotor.update(Timer.getFPGATimestamp());
        angleMotor.update(Timer.getFPGATimestamp());

        inputs.driveMotorVelocity =
                Units.rpsToMetersPerSecond(
                        driveMotor.getVelocity(), SwerveConstants.WHEEL_DIAMETER / 2);

        inputs.angleMotorAppliedVoltage = angleMotor.getAppliedVoltage();
        inputs.angle = Rotation2d.fromRotations(angleMotor.getPosition());

        inputs.moduleDistance =
                Units.rpsToMetersPerSecond(
                        driveMotor.getPosition(), SwerveConstants.WHEEL_DIAMETER / 2);
        inputs.moduleState = getModuleState();

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
        return inputs.angle;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        inputs.angleSetpoint = angle;
        angleMotor.setControl(angleControlRequest.withPosition(angle.getRotations()));
    }

    @Override
    public double getVelocity() {
        return inputs.driveMotorVelocity;
    }

    @Override
    public void setVelocity(double velocity) {
        inputs.driveMotorVelocitySetpoint = velocity;
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
                inputs.angle);
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

    @Override
    public SwerveModuleInputsAutoLogged getInputs() {
        return inputs;
    }
}
