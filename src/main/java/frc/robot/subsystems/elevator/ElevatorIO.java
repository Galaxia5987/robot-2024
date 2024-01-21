package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;

public interface ElevatorIO {

    void setPower(double power);

    void setHeight(MutableMeasure<Distance> height);

    void resetEncoder(boolean isBottom);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }

}

