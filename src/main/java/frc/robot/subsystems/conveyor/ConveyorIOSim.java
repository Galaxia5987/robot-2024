package frc.robot.subsystems.conveyor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import frc.robot.Robot;
import lib.motors.SparkMaxSim;

import java.util.Timer;

public class ConveyorIOSim implements ConveyorIO{
private final SparkMaxSim conveyor;

public ConveyorIOSim() {
    conveyor = new SparkMaxSim(1, ConveyorConstants.GEAR_RATIO, 0.5, 0);
}

    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {

    }

    @Override
    public void updateInputs() {
conveyor.update();
    }
}
