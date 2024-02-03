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
    public void updateInputs() {
        topRollerInputs.velocity.mut_replace(
                topMotor.getVelocity().getValue(), Units.RotationsPerSecond);
        topRollerInputs.voltage.mut_replace(topMotor.getMotorVoltage().getValue(), Units.Volts);

        bottomRollerInputs.velocity.mut_replace(
                bottomMotor.getVelocity().getValue(), Units.RotationsPerSecond);
        bottomRollerInputs.voltage.mut_replace(
                bottomMotor.getMotorVoltage().getValue(), Units.Volts);
    }
}
