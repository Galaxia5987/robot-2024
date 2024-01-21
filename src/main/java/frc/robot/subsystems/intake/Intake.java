package frc.robot.subsystems.intake;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Intake extends SubsystemBase {
    private IntakeIO io = null;
    Intake INSTANCE = null;

    public Intake(IntakeIO io) {
        this.io = io;
    }
    private final IntakeInputsAutoLogged inputs = io.inputs;

    public Intake getINSTANCE() {
        return INSTANCE;
    }

    public void Initialize(IntakeIO io) {
        new Intake(io);
    }

    public Command setAngle(MutableMeasure angle){
        return new InstantCommand(()-> io.setAngle(angle));
    }
   
    public Command setRollerSpeed(MutableMeasure speed){
        return new InstantCommand(()-> io.setRollerSpeed(speed));
    }

    public Command setCenterRollerSpeed(MutableMeasure speed){
        return new InstantCommand(()-> io.setCenterRoller(speed));
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Intake", inputs);
    }
}
