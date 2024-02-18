package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.Utils;
import frc.robot.lib.units.Units;

public class ModuleIOTalonFX implements ModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;

    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration angleConfig;

    private final PositionVoltage angleControlRequest =
            new PositionVoltage(0).withEnableFOC(true).withSlot(0);
    private final VelocityVoltage velocityControlRequest =
            new VelocityVoltage(0).withEnableFOC(true);
    private final SwerveModuleInputsAutoLogged inputs;

    public ModuleIOTalonFX(
            int driveMotorID,
            int angleMotorID,
            int encoderID,
            TalonFXConfiguration driveConfig,
            TalonFXConfiguration angleConfig,
            SwerveModuleInputsAutoLogged inputs) {
        this.inputs = inputs;

        this.driveMotor = new TalonFX(driveMotorID, "swerveDrive");
        this.angleMotor = new TalonFX(angleMotorID, "swerveDrive");

        this.encoder = new DutyCycleEncoder(encoderID);

        this.driveConfig = driveConfig;
        this.angleConfig = angleConfig;

        updatePID();

        this.driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setPosition(0);

        this.angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        this.angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        angleMotor.getConfigurator().apply(angleConfig);
        angleMotor.setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                driveMotor.getVelocity(),
                driveMotor.getPosition(),
                angleMotor.getPosition(),
                angleMotor.getSupplyVoltage());

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs() {
        inputs.absolutePosition = encoder.getAbsolutePosition();

        inputs.driveMotorPosition = driveMotor.getRotorPosition().getValue();
        inputs.driveMotorVelocity = Units.rpsToMetersPerSecond(
                driveMotor.getVelocity().getValue(), SwerveConstants.WHEEL_DIAMETER / 2);

        inputs.angle = Utils.normalize(Rotation2d.fromRotations(angleMotor.getPosition().getValue()));
        inputs.angleMotorAppliedVoltage = angleMotor.getSupplyVoltage().getValue();

        inputs.moduleDistance = getModulePosition().distanceMeters;
        inputs.moduleState = getModuleState();

        if (hasPIDChanged(SwerveConstants.PID_VALUES)) updatePID();
    }

    @Override
    public SwerveModuleInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public void updatePID() {
        driveConfig
                .Slot0
                .withKP(SwerveConstants.DRIVE_KP.get())
                .withKI(SwerveConstants.DRIVE_KI.get())
                .withKD(SwerveConstants.DRIVE_KD.get())
                .withKV(SwerveConstants.DRIVE_KV.get())
                .withKS(SwerveConstants.DRIVE_KS.get());
        angleConfig
                .Slot0
                .withKP(SwerveConstants.ANGLE_KP.get())
                .withKI(SwerveConstants.ANGLE_KI.get())
                .withKD(SwerveConstants.ANGLE_KD.get())
                .withKS(SwerveConstants.ANGLE_KS.get());

        driveMotor.getConfigurator().apply(driveConfig.Slot0);
        angleMotor.getConfigurator().apply(angleConfig.Slot0);
    }

    @Override
    public Rotation2d getAngle() {
        return inputs.angle;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angle = Utils.normalize(angle);
        inputs.angleSetpoint = angle;
        Rotation2d error = angle.minus(inputs.angle);
        angleControlRequest
                .withPosition(angleMotor.getPosition().getValue() + error.getRotations())
                .withFeedForward(SwerveConstants.ANGLE_KS.get())
                .withEnableFOC(true);
        angleMotor.setControl(angleControlRequest);
    }

    @Override
    public double getVelocity() {
        return inputs.driveMotorVelocity;
    }

    @Override
    public void setVelocity(double velocity) {
        inputs.driveMotorVelocitySetpoint = velocity;

        velocityControlRequest
                .withVelocity(Units.metersToRotations(velocity, SwerveConstants.WHEEL_DIAMETER / 2))
                .withEnableFOC(true);
        driveMotor.setControl(velocityControlRequest);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                Units.rpsToMetersPerSecond(
                        driveMotor.getPosition().getValue(), SwerveConstants.WHEEL_DIAMETER / 2),
                getAngle());
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    @Override
    public Command checkModule() {
        return Commands.run(
                () -> {
                    driveMotor.set(0.8);
                    angleMotor.set(0.2);
                });
    }

    @Override
    public void updateOffset(Rotation2d offset) {
        angleMotor.setPosition(encoder.getAbsolutePosition() - offset.getRotations());
    }
}
