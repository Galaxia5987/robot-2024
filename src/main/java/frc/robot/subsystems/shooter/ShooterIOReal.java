package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Ports;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final VelocityVoltage topControl = new VelocityVoltage(0);
    private final VelocityVoltage bottomControl = new VelocityVoltage(0);

    public ShooterIOReal() {
        topMotor = new TalonFX(Ports.Shooter.TOP_MOTOR_ID); // TODO: to be changed later
        bottomMotor = new TalonFX(Ports.Shooter.BOTTOM_MOTOR_ID); // TODO: to be changed later

        topMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomMotor.setNeutralMode(NeutralModeValue.Coast);

        topMotor.getConfigurator().apply(ShooterConstants.topMotorConfiguration);
        bottomMotor.getConfigurator().apply(ShooterConstants.bottomMotorConfiguration);
    }

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
        bottomRollerInputs.velocitySetpoint = ShooterConstants.STOP_POWER;
        topRollerInputs.velocitySetpoint = ShooterConstants.STOP_POWER;
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }

    @Override
    public void updateInputs() {
        topRollerInputs.velocity.mut_replace(
                topMotor.getVelocity().getValue(), Units.RotationsPerSecond);

        bottomRollerInputs.velocity.mut_replace(
                bottomMotor.getVelocity().getValue(), Units.RotationsPerSecond);
    }
}
