package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.lib.controllers.DieterController;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();
    private static SwerveDrive INSTANCE = null;
    private final SwerveModule[] modules; // FL, FR, RL, RR

    @Getter @AutoLogOutput
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final GyroIO gyro;

    @Getter
    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    SwerveConstants.WHEEL_POSITIONS[0],
                    SwerveConstants.WHEEL_POSITIONS[1],
                    SwerveConstants.WHEEL_POSITIONS[2],
                    SwerveConstants.WHEEL_POSITIONS[3]);

    @Getter private final SwerveDrivePoseEstimator estimator;
    private final SwerveDriveInputsAutoLogged loggerInputs = new SwerveDriveInputsAutoLogged();
    @Getter @AutoLogOutput private Pose2d botPose = new Pose2d();

    private boolean inCharacterizationMode = false;

    private SwerveDrive(GyroIO gyroIO, double[] wheelOffsets, ModuleIO... moduleIOs) {
        this.gyro = gyroIO;
        modules = new SwerveModule[moduleIOs.length];
        for (int i = 0; i < moduleIOs.length; i++) {
            modules[i] = new SwerveModule(moduleIOs[i], i + 1, wheelOffsets[i]);
        }

        updateModulePositions();

        estimator =
                new SwerveDrivePoseEstimator(
                        getKinematics(), getYaw(), getModulePositions(), getBotPose());
    }

    public static SwerveDrive getInstance() {
        return INSTANCE;
    }

    public static void initialize(GyroIO gyroIO, double[] offsets, ModuleIO... moduleIOs) {
        INSTANCE = new SwerveDrive(gyroIO, offsets, moduleIOs);
    }

    /**
     * Updates the offset for the gyro.
     *
     * @param angle The desired angle. [rad]
     */
    public void resetGyro(Rotation2d angle) {
        gyro.resetGyro(angle);
    }

    public void resetGyro() {
        resetGyro(new Rotation2d());
    }

    /**
     * Gets the raw yaw reading from the gyro.
     *
     * @return Yaw angle reading from gyro. [rad]
     */
    public Rotation2d getRawYaw() {
        return loggerInputs.rawYaw;
    }

    /**
     * Gets the yaw reading from the gyro with the calculated offset.
     *
     * @return Yaw angle with offset. [rad]
     */
    public Rotation2d getYaw() {
        return loggerInputs.yaw;
    }

    @AutoLogOutput
    public Rotation2d getOdometryYaw() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return getYaw().minus(Rotation2d.fromDegrees(180));
        }
        return getYaw();
    }

    /**
     * Sets the module states to the desired module states.
     *
     * @param desiredModuleStates The desired module states to set the modules to.
     */
    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        loggerInputs.desiredModuleStates = desiredModuleStates;
    }

    public void updateModulePositions() {
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = modules[i].getModulePosition();
        }
    }

    public double getVelocity() {
        return loggerInputs.linearVelocity;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return loggerInputs.currentSpeeds;
    }

    public void resetPose(Pose2d pose) {
        botPose = pose;
        resetGyro(
                pose.getRotation()
                        .minus(Constants.isRed() ? Rotation2d.fromDegrees(180) : new Rotation2d()));
        estimator.resetPosition(pose.getRotation(), modulePositions, pose);
    }

    public Command checkSwerve() {
        var command =
                Stream.of(modules)
                        .map(SwerveModule::checkModule)
                        .reduce(Commands.none(), Commands::parallel)
                        .withName("Check Swerve");
        command.addRequirements(this);
        return command;
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            modules[i].setModuleState(new SwerveModuleState(0, modules[i].getModuleState().angle));
        }
    }

    public void lock() {
        loggerInputs.desiredModuleStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(225))
                };
    }

    public void updateOffsets(double[] offsets) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateOffset(new Rotation2d(Units.rotationsToRadians(offsets[i])));
        }
    }

    /**
     * Sets the correct module states from desired chassis speeds.
     *
     * @param chassisSpeeds Desired chassis speeds.
     * @param fieldOriented Should the drive be field oriented.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldOriented) {
        loggerInputs.desiredSpeeds = chassisSpeeds;

        ChassisSpeeds fieldOrientedChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        chassisSpeeds.omegaRadiansPerSecond,
                        getYaw());

        if (new ChassisSpeeds(0, 0, 0).equals(chassisSpeeds)) {
            Arrays.stream(modules).forEach(SwerveModule::stop);
            return;
        }

        if (fieldOriented) {
            chassisSpeeds = fieldOrientedChassisSpeeds;
        }
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Sets the desired percentage of x, y and omega speeds for the frc.robot.subsystems.swerve
     *
     * @param xOutput percentage of the max possible x speed
     * @param yOutput percentage of the max possible the y speed
     * @param omegaOutput percentage of the max possible rotation speed
     */
    public void drive(double xOutput, double yOutput, double omegaOutput, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds =
                new ChassisSpeeds(
                        SwerveConstants.MAX_X_Y_VELOCITY * xOutput,
                        SwerveConstants.MAX_X_Y_VELOCITY * yOutput,
                        SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput);

        drive(chassisSpeeds, fieldOriented);
    }

    public Command driveCommand(
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            double deadband,
            BooleanSupplier fieldOriented) {
        return run(
                () ->
                        drive(
                                MathUtil.applyDeadband(forward.getAsDouble(), deadband),
                                MathUtil.applyDeadband(strafe.getAsDouble(), deadband),
                                MathUtil.applyDeadband(rotation.getAsDouble(), deadband),
                                fieldOriented.getAsBoolean()));
    }

    public Command turnCommand(MutableMeasure<Angle> rotation, double turnTolerance) {
        DieterController turnController =
                new DieterController(
                        SwerveConstants.ROTATION_KP.get(),
                        SwerveConstants.ROTATION_KI.get(),
                        SwerveConstants.ROTATION_KD.get(),
                        SwerveConstants.ROTATION_KDIETER.get());
        turnController.setTolerance(turnTolerance);
        turnController.enableContinuousInput(-0.5, 0.5);
        return run(() ->
                        drive(
                                0,
                                0,
                                turnController.calculate(
                                        estimator
                                                .getEstimatedPosition()
                                                .getRotation()
                                                .getRotations(),
                                        rotation.in(edu.wpi.first.units.Units.Rotations)),
                                false))
                .until(turnController::atSetpoint);
    }

    public Command driveAndAdjust(
            MutableMeasure<Angle> rotation,
            DoubleSupplier xJoystick,
            DoubleSupplier yJoystick,
            double deadband) {
        DieterController turnController =
                new DieterController(
                        SwerveConstants.ROTATION_KP.get(),
                        SwerveConstants.ROTATION_KI.get(),
                        SwerveConstants.ROTATION_KD.get(),
                        SwerveConstants.ROTATION_KDIETER.get());
        turnController.enableContinuousInput(-0.5, 0.5);
        turnController.setTolerance(2 / 360.0);
        return run(
                () ->
                        drive(
                                MathUtil.applyDeadband(xJoystick.getAsDouble(), deadband),
                                MathUtil.applyDeadband(yJoystick.getAsDouble(), deadband),
                                turnController.calculate(
                                        getOdometryYaw().getRotations(),
                                        rotation.in(edu.wpi.first.units.Units.Rotations)),
                                true));
    }

    public void updateSwerveInputs() {
        for (int i = 0; i < modules.length; i++) {
            loggerInputs.absolutePositions[i] = modules[i].getPosition();
            loggerInputs.currentModuleStates[i] = modules[i].getModuleState();
        }

        loggerInputs.currentSpeeds =
                kinematics.toChassisSpeeds(
                        loggerInputs.currentModuleStates[0],
                        loggerInputs.currentModuleStates[1],
                        loggerInputs.currentModuleStates[2],
                        loggerInputs.currentModuleStates[3]);

        loggerInputs.linearVelocity =
                Math.hypot(
                        loggerInputs.currentSpeeds.vxMetersPerSecond,
                        loggerInputs.currentSpeeds.vyMetersPerSecond);
    }

    public void updateGyroInputs() {
        loggerInputs.rawYaw = gyro.getRawYaw();
        loggerInputs.yaw = gyro.getYaw();
        gyro.updateInputs(loggerInputs);
    }

    public double getAcceleration() {
        return loggerInputs.acceleration;
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.updateInputs();
        }
        updateGyroInputs();
        updateSwerveInputs();
        updateModulePositions();
        estimator.update(getOdometryYaw(), getModulePositions());
        botPose = estimator.getEstimatedPosition();

        SwerveDriveKinematics.desaturateWheelSpeeds(
                loggerInputs.desiredModuleStates, SwerveConstants.MAX_X_Y_VELOCITY);
        if (!inCharacterizationMode) {
            for (int i = 0; i < modules.length; i++) {
                modules[i].setModuleState(loggerInputs.desiredModuleStates[i]);
            }
        }

        Logger.processInputs("SwerveDrive", loggerInputs);
    }

    public Command characterize() {
        SysIdRoutine routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> volts) -> {
                                    for (SwerveModule module : modules) {
                                        module.characterize(
                                                volts.in(edu.wpi.first.units.Units.Volts));
                                    }
                                },
                                (SysIdRoutineLog log) -> {
                                    for (SwerveModule module : modules) {
                                        module.updateSysIdRoutineLog(log);
                                    }
                                },
                                this));
        return Commands.sequence(
                        Commands.runOnce(() -> inCharacterizationMode = true),
                        routine.dynamic(SysIdRoutine.Direction.kForward),
                        Commands.waitSeconds(1),
                        routine.dynamic(SysIdRoutine.Direction.kReverse),
                        Commands.waitSeconds(1),
                        routine.quasistatic(SysIdRoutine.Direction.kForward),
                        Commands.waitSeconds(1),
                        routine.quasistatic(SysIdRoutine.Direction.kReverse))
                .finallyDo(() -> inCharacterizationMode = false);
    }
}
