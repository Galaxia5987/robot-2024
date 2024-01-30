package frc.robot.subsystems.conveyor;

import static frc.robot.subsystems.conveyor.ConveyorConstants.FEED_VELOCITY;
import static frc.robot.subsystems.conveyor.ConveyorConstants.SETPOINT_TOLERANCE;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
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

    public Command setVelocity(Supplier<MutableMeasure<Velocity<Angle>>> velocity) {
        return run(() -> io.setVelocity(velocity.get()));
    }

    public Command feed() {
        return setVelocity(() -> FEED_VELOCITY);
    }

    public boolean readyToFeed() {
        return inputs.velocitySetpoint.isNear(inputs.velocity, SETPOINT_TOLERANCE.in(Units.Value));
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Conveyor", inputs);
    }
}
