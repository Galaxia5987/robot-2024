package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.SparkMaxSim;

import static frc.robot.subsystems.conveyor.ConveyorConstants.*;

public class ConveyorIOSim implements ConveyorIO {
    private final SparkMaxSim conveyor;

    public static PIDController controller = new PIDController(KP.get(), KI.get(), KD.get(), 0.02);

    public static SimpleMotorFeedforward feed = new SimpleMotorFeedforward(KS.get(), KV.get(), KA.get());



    public ConveyorIOSim() {
        conveyor = new SparkMaxSim(1, GEAR_RATIO, 0.00005, 1);
        conveyor.setController(controller);
    }

    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        conveyor.setReference(velocity.in(Units.RotationsPerSecond), CANSparkMax.ControlType.kVelocity);
//        conveyor.set(0.5);
    }

    @Override
    public void updateInputs() {
        conveyor.update(Timer.getFPGATimestamp());
        inputs.velocity.mut_replace(Units.RotationsPerSecond.of(conveyor.getVelocity()));
        inputs.appliedVoltage.mut_replace(Units.Volts.of(conveyor.getBusVoltage()));
        inputs.appliedCurrent.mut_replace(Units.Amps.of(conveyor.getOutputCurrent()));
    }
}
