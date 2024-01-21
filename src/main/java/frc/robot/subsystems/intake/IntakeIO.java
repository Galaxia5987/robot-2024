package frc.robot.subsystems.intake;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {

    default boolean hasNote(){return false;}
    default Command setAngle(MutableMeasure angle){return new InstantCommand(); }// [Rad]
    default Command setRollerSpeed(MutableMeasure speed){return new InstantCommand();} //[Rps]
    default Command setCenterRoller(MutableMeasure speed){return new InstantCommand();}
    void update();

    @AutoLog
    class IntakeInputs {
        MutableMeasure<Angle> currentAngle  = MutableMeasure.zero(Radians);
        MutableMeasure<Angle> angleSetPoint  = MutableMeasure.zero(Radians);
        MutableMeasure<Velocity<Angle>> rollerSpeed  = MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Velocity<Angle>> centerRollerSpeed  = MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Voltage> angleMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> spinMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> centerMotorVoltage = MutableMeasure.zero(Volts);
        boolean hasNote = false;

    }
}
