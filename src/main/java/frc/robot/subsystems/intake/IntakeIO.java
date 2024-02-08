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

    void setRollerSpeed(MutableMeasure<Velocity<Angle>> speed);

    void setCenterRollerSpeed(double speed);

    void updateInputs();

    @AutoLog
    class IntakeInputs {
        MutableMeasure<Angle> currentAngle = MutableMeasure.zero(Radians);
        MutableMeasure<Angle> angleSetpoint = MutableMeasure.zero(Radians);
        MutableMeasure<Velocity<Angle>> currentRollerSpeed =
                MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Velocity<Angle>> rollerSpeedSetpoint =
                MutableMeasure.zero(RotationsPerSecond);
        MutableMeasure<Velocity<Angle>> currentCenterRollerSpeed =
                MutableMeasure.zero(RotationsPerSecond);
        double centerRollerSpeedSetpoint = 0;
        MutableMeasure<Voltage> angleMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> spinMotorVoltage = MutableMeasure.zero(Volts);
        MutableMeasure<Voltage> centerMotorVoltage = MutableMeasure.zero(Volts);
    }
}
