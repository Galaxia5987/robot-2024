package frc.robot.subsystems.intake;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    Intake INSTANCE = null;
    private IntakeIO io = null;
    private final IntakeInputsAutoLogged inputs = io.inputs;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Intake getINSTANCE() {
        return INSTANCE;
    }

    public void Initialize(IntakeIO io) {
        new Intake(io);
    }

    public Command setAngle(MutableMeasure angle) {
        return new InstantCommand(() -> io.setAngle(angle));
    }

    public Command setAngle(IntakePose intakePose) {
        if (intakePose == IntakePose.UP) {
            setAngle((MutableMeasure) Units.Radians.of(IntakeConstants.DOWN_ANGLE));
        }
        else{
            setAngle((MutableMeasure) Units.Radians.of(IntakeConstants.UP_ANGLE));
        }
        return new InstantCommand();
    }

    public Command setRollerSpeed(MutableMeasure speed) {
        return new InstantCommand(() -> io.setRollerSpeed(speed));
    }

    public Command setCenterRollerSpeed(MutableMeasure speed) {
        return new InstantCommand(() -> io.setCenterRoller(speed));
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Intake", inputs);
    }

    enum IntakePose {
        UP,
        DOWN
    }
}
