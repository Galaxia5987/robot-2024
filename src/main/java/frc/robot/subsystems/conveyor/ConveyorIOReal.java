package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;

public class ConveyorIOReal implements ConveyorIO {

    private CANSparkMax roller = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

    public ConveyorIOReal() {
        roller.restoreFactoryDefaults();
        roller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        roller.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        roller.setInverted(true);
        for (int i = 1; i <= 6; i++) {
            roller.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.fromId(i), 50_000);
        }
    }


    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {

    }

    @Override
    public void updateInputs() {
        inputs.currentVelocity.mut_replace(Units.RotationsPerSecond.of(roller.getEncoder().getVelocity()).mutableCopy());
        inputs.appliedCurrent.mut_replace(Units.Amps.of(roller.getOutputCurrent()).mutableCopy());
        inputs.appliedVoltage.mut_replace(Units.Volts.of(roller.getBusVoltage()).mutableCopy());
        roller.burnFlash();
    }
}
