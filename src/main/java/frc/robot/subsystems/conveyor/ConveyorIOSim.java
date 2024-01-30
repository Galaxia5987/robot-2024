package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.SparkMaxSim;

public class ConveyorIOSim implements ConveyorIO {
    private final SparkMaxSim conveyor;

    public ConveyorIOSim() {
        conveyor = new SparkMaxSim(1, ConveyorConstants.GEAR_RATIO, 0.5, ConveyorConstants.GEAR_RATIO);
        conveyor.setController(ConveyorConstants.CONTROLLER);
        conveyor.setProfiledController();
    }

    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        conveyor.setReference(velocity.in(Units.RotationsPerSecond), CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void updateInputs() {
        conveyor.update(Timer.getFPGATimestamp());
        inputs.velocity.mut_replace(Units.RotationsPerSecond.of(conveyor.getVelocity()));
        inputs.appliedVoltage.mut_replace(Units.Volts.of(conveyor.getBusVoltage()));
        inputs.appliedCurrent.mut_replace(Units.Amps.of(conveyor.getAppliedOutput()));
    }
}
