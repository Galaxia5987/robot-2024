package frc.robot.subsystems.conveyor;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Ports;

public class ConveyorIOReal implements ConveyorIO {

    private final TalonFX roller = new TalonFX(Ports.Conveyor.MOTOR_ID);
    private final VelocityVoltage control = new VelocityVoltage(0).withEnableFOC(true);

    public ConveyorIOReal() {
        roller.getConfigurator().apply(ConveyorConstants.MOTOR_CONFIG);
    }

    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        roller.setControl(control.withVelocity(velocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void stop() {
        roller.stopMotor();
    }

    @Override
    public void updateInputs() {
        inputs.currentVelocity.mut_replace(
                roller.getVelocity().getValue(), Units.RotationsPerSecond);
    }
}
