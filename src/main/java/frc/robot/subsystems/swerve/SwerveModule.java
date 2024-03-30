package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

    private final SwerveModuleInputsAutoLogged inputs;

    private final ModuleIO io;

    private final int number;
    private final Timer timer = new Timer();
    private final double offset;

    public SwerveModule(ModuleIO io, int number, double offset) {
        this.io = io;
        this.number = number;
        this.offset = offset;

        inputs = io.getInputs();

        timer.start();
        timer.reset();
    }

    public void setVelocity(double velocity) {
        var angleError = inputs.angleSetpoint.minus(inputs.angle);
        velocity *= angleError.getCos();
        io.setVelocity(velocity);
    }

    /**
     * Gets the state of a module.
     *
     * @return The state of a module.
     */
    public SwerveModuleState getModuleState() {
        return io.getModuleState();
    }

    /**
     * Sets the module to a desired module state.
     *
     * @param moduleState A module state to set the module to.
     */
    public void setModuleState(SwerveModuleState moduleState) {
        moduleState = SwerveModuleState.optimize(moduleState, inputs.angle);
        setVelocity(moduleState.speedMetersPerSecond);
        io.setAngle(moduleState.angle);
    }

    /**
     * Gets the position of the module.
     *
     * @return Position of the module.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(inputs.moduleDistance, inputs.angle);
    }

    /**
     * Gets the position of the absolute encoder.
     *
     * @return Position of the absolute encoder. [sensor ticks]
     */
    public double getPosition() {
        return inputs.absolutePosition;
    }

    /**
     * Updates the position of the angle motor with an offset and an absolute encoder.
     *
     * @param offset The offset to update the angle motor's position. [sensor ticks]
     */
    public void updateOffset(Rotation2d offset) {
        io.updateOffset(offset);
    }

    public void stop() {
        io.stop();
    }

    public Command checkModule() {
        return io.checkModule();
    }

    public void updateInputs() {
        io.updateInputs();
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs("module_" + number, inputs);
        }
    }

    public double getAcceleration() {
        return inputs.driveMotorAcceleration;
    }

    @Override
    public void periodic() {
        if (timer.advanceIfElapsed(2)) {
            io.updateOffset(new Rotation2d(Units.rotationsToRadians(offset)));
        }
    }

    public void characterize(double voltage) {
        io.setVoltage(voltage);
        io.setAngle(new Rotation2d());
    }

    public void updateSysIdRoutineLog(SysIdRoutineLog log) {
        log.motor("" + number)
                .voltage(edu.wpi.first.units.Units.Volts.of(inputs.angleMotorAppliedVoltage))
                .angularPosition(
                        edu.wpi.first.units.Units.Rotations.of(inputs.angle.getRotations()))
                .angularVelocity(
                        edu.wpi.first.units.Units.RotationsPerSecond.of(
                                inputs.driveMotorVelocity
                                        / (SwerveConstants.WHEEL_DIAMETER * Math.PI)));
    }
}
