package frc.robot.subsystems.conveyor;

import static frc.robot.subsystems.conveyor.ConveyorConstants.FEED_POWER;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {

    private static Conveyor INSTANCE = null;

    private final ConveyorIO io;
    private final ConveyorInputsAutoLogged inputs = ConveyorIO.inputs;

    private Conveyor(ConveyorIO io) {
        this.io = io;
    }

    public static Conveyor getInstance() {
        return INSTANCE;
    }

    public static void initialize(ConveyorIO io) {
        INSTANCE = new Conveyor(io);
    }

    public Command setPower(DoubleSupplier power) {
        return run(() -> io.setPower(power.getAsDouble()));
    }

    public Command feed(BooleanSupplier feed) {
        return setPower(() -> feed.getAsBoolean() ? FEED_POWER : 0);
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Conveyor", inputs);
    }
}
