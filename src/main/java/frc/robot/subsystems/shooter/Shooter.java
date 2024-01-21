package frc.robot.subsystems.shooter;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import lib.Utils;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static Shooter INSTANCE = null;
    private final ShooterInputsAutoLogged inputs = ShooterIO.inputs;
    private final ShooterIO io;

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

    /**
     * Sets the velocity of the shooter.
     *
     * @param velocity The velocity of the Shooter to set.
     */
    public Command setVelocity(Supplier<Measure<Velocity<Angle>>> velocity) {
        return run(() -> inputs.velocitySetpoint.mut_replace(velocity.get()));
    }

    public MutableMeasure<Velocity<Angle>> getVelocity() {
        return inputs.velocity;
    }

    public boolean atSetpoint() {
        return inputs.velocity.isNear(inputs.velocitySetpoint, ShooterConstants.atSetpointTolerance);
    }

    /** Updates the state of the shooter. */
    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        io.setVelocity(inputs.velocitySetpoint);
    }
}
