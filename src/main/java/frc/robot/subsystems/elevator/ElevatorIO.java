package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;

public interface ElevatorIO {
    ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    void setPower(double power);

    void setHeight(MutableMeasure<Distance> height);

    void updateInputs(ElevatorInputs inputs);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }
}
