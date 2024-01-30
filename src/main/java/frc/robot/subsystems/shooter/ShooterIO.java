package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    TopRollerInputsAutoLogged topRollerInputs = new TopRollerInputsAutoLogged();
    BottomRollerInputsAutoLogged bottomRollerInputs = new BottomRollerInputsAutoLogged();

    void setTopVelocity(MutableMeasure<Velocity<Angle>> velocity);

    void setBottomVelocity(MutableMeasure<Velocity<Angle>> velocity);

    void stop();

    /** Update the inputs of the Shooter */
    void updateInputs();

    class RollerInputs {
        public MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> velocitySetpoint =
                MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);

        public RollerInputs() {}
    }
}
