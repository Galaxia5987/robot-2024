package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

public class ShooterIOReal implements ShooterIO {
    @Override
    public void setTopVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        topRollerInputs.velocitySetpoint.mut_replace(velocity);
        topMotor.setControl(topControl.withVelocity(velocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void setBottomVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        bottomRollerInputs.velocitySetpoint.mut_replace(velocity);
        bottomMotor.setControl(bottomControl.withVelocity(velocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void stop() {
        bottomRollerInputs.velocitySetpoint = Units.RotationsPerSecond.zero().mutableCopy();
        topRollerInputs.velocitySetpoint = Units.RotationsPerSecond.zero().mutableCopy();
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }

    @Override
    public void updateInputs() {}
}
