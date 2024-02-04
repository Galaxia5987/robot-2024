package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

public class ConveyorIOReal implements ConveyorIO{

private CANSparkMax roller = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

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
