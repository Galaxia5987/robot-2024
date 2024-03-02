package frc.robot.subsystems.climb;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;

public interface ClimbIO {
    ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    void setPower(double power);

    void setHeight(MutableMeasure<Distance> height);

    void openStopper();

    void closeStopper();

    void manualReset();

    void updateInputs(ClimbInputs inputs);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION
    }
}
