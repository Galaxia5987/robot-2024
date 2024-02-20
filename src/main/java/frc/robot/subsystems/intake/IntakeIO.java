package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    void setAngle(MutableMeasure<Angle> angle);

    void setRollerSpeed(double speed);

    void setCenterRollerSpeed(double speed);

    void reset(Measure<Angle> angle);

    void updateInputs();

    @AutoLog
    class IntakeInputs {
        MutableMeasure<Angle> currentAngle = MutableMeasure.zero(Radians);
        MutableMeasure<Angle> angleSetpoint = MutableMeasure.zero(Radians);
        double rollerSpeedSetpoint = 0;
        double centerRollerSpeedSetpoint = 0;
        MutableMeasure<Voltage> angleMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> spinMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> centerMotorVoltage = MutableMeasure.zero(Volts);
    }
}
