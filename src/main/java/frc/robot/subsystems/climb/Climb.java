package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

    private static Climb INSTANCE;

    private final ClimbInputsAutoLogged inputs = ClimbIO.inputs;
    private final ClimbIO io;
    private final Timer timer = new Timer();

    private Climb(ClimbIO io) {
        this.io = io;

        timer.start();
        timer.reset();
    }

    public static Climb getInstance() {
        return INSTANCE;
    }

    public static void initialize(ClimbIO io) {
        INSTANCE = new Climb(io);
    }

    public Command lock() {
        return Commands.run(io::closeStopper).withTimeout(0.5).andThen(io::disableStopper);
    }

    public Command unlock() {
        return Commands.run(io::openStopper).withTimeout(0.5).andThen(io::disableStopper);
    }

    public Command setPower(DoubleSupplier power) {
        return Commands.parallel(run(() -> io.setPower(power.getAsDouble())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (timer.advanceIfElapsed(0.0)) {
            Logger.processInputs(this.getClass().getSimpleName(), inputs);
        }
    }
}
