package frc.robot.subsystems.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE = null;
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = IntakeIO.inputs;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Intake getInstance() {
        return INSTANCE;
    }

    public void initialize(IntakeIO io) {
        INSTANCE = new Intake(io);
    }

    public Command setAngle(MutableMeasure<Angle> angle) {
        return runOnce(() -> io.setAngle(angle));
    }

    public Command setAngle(IntakeConstants.IntakePose intakePose) {
        if (intakePose == IntakeConstants.IntakePose.UP) {
            return setAngle(IntakeConstants.IntakePose.UP.intakePose);
        } else {
            return setAngle(IntakeConstants.IntakePose.DOWN.intakePose);
        }
    }

    public Command setRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        return runOnce(() -> io.setRollerSpeed(speed));
    }

    public Command setCenterRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        return runOnce(() -> io.setCenterRollerSpeed(speed));
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Intake", inputs);
    }
}
