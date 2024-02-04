package frc.robot.subsystems.conveyor;

import static frc.robot.subsystems.conveyor.ConveyorConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import lib.motors.SparkMaxSim;

public class ConveyorIOSim implements ConveyorIO {
    private final SparkMaxSim conveyor;

    public static PIDController controller = new PIDController(KP.get(), KI.get(), KD.get(), 0.02);

    public static SimpleMotorFeedforward feed =
            new SimpleMotorFeedforward(KS.get(), KV.get(), KA.get());

    public ConveyorIOSim() {
        conveyor = new SparkMaxSim(1, GEAR_RATIO, 0.000_05, 1);
        conveyor.setController(controller);
    }

    @Override
    public void setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        conveyor.setReference(
                velocity.in(Units.RotationsPerSecond),
                CANSparkMax.ControlType.kVelocity,
                feed.calculate(velocity.in(Units.RotationsPerSecond)));
    }

    @Override
    public void updateInputs() {
        conveyor.update(Timer.getFPGATimestamp());
        inputs.currentVelocity.mut_replace((conveyor.getVelocity()), Units.RotationsPerSecond);
        inputs.appliedVoltage.mut_replace((conveyor.getBusVoltage()), Units.Volts);
        inputs.appliedCurrent.mut_replace((conveyor.getOutputCurrent()), Units.Amps);
    }
}
