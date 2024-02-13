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
import frc.robot.lib.PhoenixOdometryThread;
import java.util.Queue;
import lib.Utils;
import lib.units.Units;

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
    private final Queue<Double> distanceQueue;
    private final Queue<Double> angleQueue;
    private final Queue<Double> timestampQueue;
    private Rotation2d angleSetpoint = new Rotation2d();
    private Rotation2d currentAngle = new Rotation2d();
    private double driveMotorVelocitySetpoint = 0;

    public ModuleIOTalonFX(
            int driveMotorID,
            int angleMotorID,
            int encoderID,
            TalonFXConfiguration driveConfig,
            TalonFXConfiguration angleConfig) {

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

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);

        var drivePositionSignal = driveMotor.getPosition();
        var driveVelocitySignal = driveMotor.getVelocity();
        distanceQueue =
                PhoenixOdometryThread.getInstance()
                        .registerSignal(driveMotor, drivePositionSignal, driveVelocitySignal);

        var anglePositionSignal = angleMotor.getPosition();
        var angleVelocitySignal = angleMotor.getVelocity();
        angleQueue =
                PhoenixOdometryThread.getInstance()
                        .registerSignal(angleMotor, anglePositionSignal, angleVelocitySignal);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        BaseStatusSignal.setUpdateFrequencyForAll(
                SwerveConstants.ODOMETRY_FREQUENCY,
                drivePositionSignal,
                anglePositionSignal,
                driveVelocitySignal,
                angleVelocitySignal);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.absolutePosition = encoder.getAbsolutePosition();

        inputs.driveMotorSupplyCurrent = driveMotor.getSupplyCurrent().getValue();
        inputs.driveMotorStatorCurrent = driveMotor.getStatorCurrent().getValue();
        inputs.driveMotorPosition = driveMotor.getRotorPosition().getValue();
        inputs.driveMotorVelocity = getVelocity();
        inputs.driveMotorVelocitySetpoint = driveMotorVelocitySetpoint;

        inputs.angleMotorSupplyCurrent = angleMotor.getSupplyCurrent().getValue();
        inputs.angleMotorStatorCurrent = angleMotor.getStatorCurrent().getValue();
        inputs.angleMotorPosition = angleMotor.getRotorPosition().getValue();
        inputs.angleMotorVelocity = angleMotor.getVelocity().getValue();

        inputs.angle = getAngle();
        currentAngle = inputs.angle;

        inputs.angleSetpoint = angleSetpoint;

        inputs.moduleDistance = getModulePosition().distanceMeters;
        inputs.moduleState = getModuleState();

        inputs.highFreqDistances = distanceQueue.stream().mapToDouble((Double d) -> d).toArray();
        inputs.highFreqAngles = angleQueue.stream().mapToDouble((Double d) -> d).toArray();
        inputs.highFreqTimestamps = timestampQueue.stream().mapToDouble((Double d) -> d).toArray();

        distanceQueue.clear();
        angleQueue.clear();
        timestampQueue.clear();

        if (hasPIDChanged(SwerveConstants.PID_VALUES)) updatePID();
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
        return Utils.normalize(Rotation2d.fromRotations(angleMotor.getPosition().getValue()));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angle = Utils.normalize(angle);
        angleSetpoint = angle;
        Rotation2d error = angle.minus(currentAngle);
        angleControlRequest
                .withPosition(angleMotor.getPosition().getValue() + error.getRotations())
                .withFeedForward(SwerveConstants.ANGLE_KS.get())
                .withEnableFOC(true);
        angleMotor.setControl(angleControlRequest);
    }

    @Override
    public double getVelocity() {
        return Units.rpsToMetersPerSecond(
                driveMotor.getVelocity().getValue(), SwerveConstants.WHEEL_DIAMETER / 2);
    }

    @Override
    public void setVelocity(double velocity) {
        driveMotorVelocitySetpoint = velocity;

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
