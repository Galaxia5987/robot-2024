package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    default void setAngle(MutableMeasure<Angle> angle) {}

    default void setRollerSpeed(double speed) {}

    default void setCenterRollerSpeed(double speed) {}

    default void reset(Measure<Angle> angle) {}

    default void setAnglePower(double power) {}

    default void updateInputs() {}

    @AutoLog
    class IntakeInputs {
        MutableMeasure<Angle> currentAngle =
                Degrees.of(110).mutableCopy(); // TODO: check default value
        MutableMeasure<Angle> angleSetpoint = MutableMeasure.zero(Radians);
        double rollerSpeedSetpoint = 0;
        double centerRollerSpeedSetpoint = 0;
        MutableMeasure<Voltage> angleMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> spinMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> centerMotorVoltage = MutableMeasure.zero(Volts);
    }
}
