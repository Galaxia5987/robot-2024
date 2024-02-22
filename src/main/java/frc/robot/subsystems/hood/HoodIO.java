package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

    void updateInternalEncoder();

    void setAngle(MutableMeasure<Angle> angle);

    /** Update the inputs of the hood */
    void updateInputs();

    @AutoLog
    class HoodInputs {
        public MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
        public MutableMeasure<Angle> angleSetpoint = MutableMeasure.zero(Rotations);
        public MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        public MutableMeasure<Angle> absoluteEncoderAngle = MutableMeasure.zero(Rotations);
        public MutableMeasure<Angle> absoluteEncoderAngleNoOffset = MutableMeasure.zero(Rotations);
    }
}
