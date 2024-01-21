package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
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
    public Command setVelocity(Supplier<Double> velocity) {
        return run(() -> inputs.velocitySetpoint.mut_replace(velocity.get(), RotationsPerSecond));
    }

    public MutableMeasure<Velocity<Angle>> getVelocity() {
        return inputs.velocity;
    }

    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    /** Updates the state of the shooter. */
    @Override
    public void periodic() {
        io.setVelocity(inputs.velocitySetpoint);

        io.updateInputs();
        Logger.processInputs("Shooter", inputs);
    }
}
