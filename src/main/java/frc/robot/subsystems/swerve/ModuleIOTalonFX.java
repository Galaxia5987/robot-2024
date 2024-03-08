package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

    private final MotionMagicVoltage angleControlRequest =
            new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
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
                200,
                driveMotor.getVelocity(),
                driveMotor.getPosition(),
                angleMotor.getPosition(),
                angleMotor.getMotorVoltage());

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs() {
        inputs.absolutePosition = encoder.getAbsolutePosition();

        inputs.driveMotorPosition = driveMotor.getPosition().getValue();
        inputs.driveMotorVelocity =
                Units.rpsToMetersPerSecond(
                        driveMotor.getVelocity().getValue(), SwerveConstants.WHEEL_DIAMETER / 2);
        inputs.driveMotorVoltage = driveMotor.getMotorVoltage().getValue();

        inputs.angle =
                Utils.normalize(Rotation2d.fromRotations(angleMotor.getPosition().getValue()));
        inputs.angleMotorAppliedVoltage = angleMotor.getMotorVoltage().getValue();
        inputs.angleMotorVelocity = angleMotor.getVelocity().getValue();

        inputs.moduleDistance =
                Units.rpsToMetersPerSecond(
                        inputs.driveMotorPosition, SwerveConstants.WHEEL_DIAMETER / 2);
        inputs.moduleState = getModuleState();

        inputs.encoderConnected = encoder.isConnected();
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
                .withPosition(inputs.angle.getRotations() + error.getRotations())
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
        return new SwerveModulePosition(inputs.moduleDistance, getAngle());
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

    public void setVoltage(double volts) {
        driveMotor.setControl(new VoltageOut(volts));
    }
}
