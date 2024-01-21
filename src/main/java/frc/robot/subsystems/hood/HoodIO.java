package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

public interface HoodIO {
    HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

    void updateInternalEncoder();

    void resetAbsoluteEncoder();

    void setAngle(MutableMeasure<Angle> angle);

    void setPower(double power);

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
