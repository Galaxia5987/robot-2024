package frc.robot.subsystems.conveyor;

import static frc.robot.subsystems.conveyor.ConveyorConstants.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {

    private static Conveyor INSTANCE = null;

    private final ConveyorIO io;
    private final ConveyorInputsAutoLogged inputs = ConveyorIO.inputs;
    private final Timer timer = new Timer();

    private Conveyor(ConveyorIO io) {
        this.io = io;

        timer.start();
        timer.reset();
    }

    public static Conveyor getInstance() {
        return INSTANCE;
    }

    public static void initialize(ConveyorIO io) {
        INSTANCE = new Conveyor(io);
    }

    public Command setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        return run(
                () -> {
                    inputs.velocitySetpoint.mut_replace(velocity);
                    io.setVelocity(velocity);
                });
    }

    public Command feed() {
        return setVelocity(FEED_VELOCITY);
    }

    @AutoLogOutput
    public boolean readyToFeed() {
        return inputs.velocitySetpoint.isNear(
                inputs.currentVelocity, SETPOINT_TOLERANCE.in(Units.Value));
    }

    public Command stop() {
        return setVelocity(STOP_VELOCITY);
    }

    @Override
    public void periodic() {
        io.updateInputs();
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs("Conveyor", inputs);
        }
    }
}
