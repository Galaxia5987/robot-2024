package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

    /**
     * Set the position of the hood
     *
     * @param angle The position of the subsystem.
     */
    void setAngle(Measure<Angle> angle);

    void updateInternalEncoder();

    void resetAbsoluteEncoder();

    boolean atSetpoint();

    /** Update the inputs of the hood */
    void updateInputs();

    @AutoLog
    class HoodInputs {
        public MutableMeasure<Angle> angle = MutableMeasure.zero(Radians);
        public MutableMeasure<Angle> angleSetpoint = MutableMeasure.zero(Radians);
        public MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        public MutableMeasure<Angle> absoluteEncoderAngle = MutableMeasure.zero(Radians);
    }
}
