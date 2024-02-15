package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import frc.robot.Ports;

public class ConveyorIOReal implements ConveyorIO {

    private CANSparkMax roller = new CANSparkMax(Ports.Conveyor.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    public ConveyorIOReal() {
        roller.restoreFactoryDefaults();
        roller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        roller.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE.in(Units.Volts));
        roller.setInverted(true);
        roller.burnFlash();
    }

    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        roller.set(velocity.in(Units.RotationsPerSecond));
    }

    @Override
    public void updateInputs() {
        inputs.currentVelocity.mut_replace(
                (roller.getEncoder().getVelocity()), Units.RotationsPerSecond);
        inputs.appliedCurrent.mut_replace((roller.getOutputCurrent()), Units.Amps);
        inputs.appliedVoltage.mut_replace((roller.getBusVoltage()), Units.Volts);
    }
}
