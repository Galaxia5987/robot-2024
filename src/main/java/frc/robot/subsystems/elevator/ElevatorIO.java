package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;

public interface ElevatorIO {
    ElevatorInputs inputs = new ElevatorInputs();

    void setPower(double power);

    void setHeight(MutableMeasure<Distance> height);

    void resetEncoder(boolean isBottom);

    void updateInputs(ElevatorInputs inputs);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }
}
