package frc.robot.subsystems.shooter;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static Shooter INSTANCE = null;
    private final ShooterInputsAutoLogged inputs = ShooterIO.inputs;
    private final ShooterIO io;

    private final String SUBSYSTEM_NAME = this.getClass().getSimpleName();

    /**
     * Constructor for Shooter subsystem.
     *
     * @param io IO of the subsystem.
     */
    private Shooter(ShooterIO io) {
        this.io = io;
    }

    /**
     * Gets the single instance of Shooter.
     *
     * @return The single instance of Shooter.
     */
    public static Shooter getInstance() {
        return INSTANCE;
    }

    public static void initialize(ShooterIO io) {
        INSTANCE = new Shooter(io);
    }

    public Command setVelocity(
            Supplier<MutableMeasure<Velocity<Angle>>> topVelocity,
            Supplier<MutableMeasure<Velocity<Angle>>> bottomVelocity) {
        return run(() -> {
                    io.setTopVelocity(topVelocity.get());
                    io.setBottomVelocity(bottomVelocity.get());
                })
                .withName("Set Shooter Velocity");
    }

    public Command setVelocity(Supplier<MutableMeasure<Velocity<Angle>>> velocity) {
        return setVelocity(velocity, velocity);
    }

    public Command stop() {
        return setVelocity(() -> ShooterConstants.STOP_POWER);
    }

    @AutoLogOutput
    public boolean atSetpoint() {
        return inputs.topVelocity.isNear(
                        inputs.topVelocitySetpoint, ShooterConstants.SETPOINT_TOLERANCE_TOP)
                && inputs.bottomVelocity.isNear(
                        inputs.bottomVelocitySetpoint, ShooterConstants.SETPOINT_TOLERANCE_BOTTOM);
    }

    /** Updates the state of the shooter. */
    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs(SUBSYSTEM_NAME, inputs);
    }
}
