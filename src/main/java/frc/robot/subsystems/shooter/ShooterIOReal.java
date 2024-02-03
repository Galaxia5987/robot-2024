package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

public class ShooterIOReal implements ShooterIO {
    @Override
    public void setTopVelocity(MutableMeasure<Velocity<Angle>> velocity) {}

    @Override
    public void setBottomVelocity(MutableMeasure<Velocity<Angle>> velocity) {}

    @Override
    public void stop() {}

    @Override
    public void updateInputs() {}
}
