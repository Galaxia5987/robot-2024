package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;

public class ElevatorIOReal implements ElevatorIO {
    @Override
    public void setPower(double power) {}

    @Override
    public void setHeight(MutableMeasure<Distance> height) {}

    @Override
    public void updateInputs(ElevatorInputs inputs) {}
}
