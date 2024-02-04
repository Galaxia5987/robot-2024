package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    void setAngle(MutableMeasure<Angle> angle);

    void setRollerSpeed(double speed);

    void setCenterRollerSpeed(double speed);

    void updateInputs();

    @AutoLog
    class IntakeInputs {
        MutableMeasure<Angle> currentAngle = MutableMeasure.zero(Radians);
        MutableMeasure<Angle> angleSetPoint = MutableMeasure.zero(Radians);
        MutableMeasure<Velocity<Angle>> currentRollerSpeed =
                MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Velocity<Angle>> rollerSpeedSetPoint =
                MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Velocity<Angle>> currentCenterRollerSpeed =
                MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Velocity<Angle>> centerRollerSpeedSetPoint =
                MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Voltage> angleMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> spinMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> centerMotorVoltage = MutableMeasure.zero(Volts);
    }
}
