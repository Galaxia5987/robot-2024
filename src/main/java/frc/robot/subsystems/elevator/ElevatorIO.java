package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;

public interface ElevatorIO {

    void setPower(double power);

    void getPower();

    void setHeight(MutableMeasure<Distance> height);

    void getHeight();

    void getTopSensor();

    void getBottomSensor();

    void getControlMode();

    void resetEncoder(boolean isBottom);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }

}

