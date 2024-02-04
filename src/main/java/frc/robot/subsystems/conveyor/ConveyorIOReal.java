package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;

public class ConveyorIOReal implements ConveyorIO {

    private CANSparkMax roller = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

    public ConveyorIOReal() {
        roller.restoreFactoryDefaults();
        roller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        roller.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        roller.setInverted(true);
        for (int i = 0; i < ; i++) {
            
        }
    }


    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {

    }

    @Override
    public void updateInputs() {
        inputs.velocitySetpoint;
        inputs.appliedCurrent;
        inputs.appliedVoltage;
    }
}
