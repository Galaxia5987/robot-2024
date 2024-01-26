package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

    void updateInternalEncoder();

    void setAngle(MutableMeasure<Angle> angle);

    void setPower(double power);

    /** Update the inputs of the hood */
    void updateInputs();

    @AutoLog
    class HoodInputs {
        public MutableMeasure<Angle> angle = MutableMeasure.zero(Radians);
        public MutableMeasure<Angle> angleSetpoint = MutableMeasure.zero(Radians);
        public MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        public double powerSetpoint;
        public MutableMeasure<Angle> absoluteEncoderAngle = MutableMeasure.zero(Radians);
        public Mode controlMode;
    }

    enum Mode {
        POWER,
        ANGLE
    }
}
