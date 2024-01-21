package frc.robot.subsystems.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {

    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    default void setAngle(MutableMeasure angle) {
    } // [Rad]

    default void setRollerSpeed(MutableMeasure speed) {
    } // [Rps]

    default void setCenterRoller(MutableMeasure speed) {
    }

    void updateInputs();

    @AutoLog
    class IntakeInputs {
        MutableMeasure<Angle> currentAngle = MutableMeasure.zero(Radians);
        MutableMeasure<Angle> angleSetPoint = MutableMeasure.zero(Radians);
        MutableMeasure<Velocity<Angle>> rollerSpeed = MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Velocity<Angle>> centerRollerSpeed = MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Voltage> angleMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> spinMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> centerMotorVoltage = MutableMeasure.zero(Volts);
        boolean hasNote = false;
    }
}
