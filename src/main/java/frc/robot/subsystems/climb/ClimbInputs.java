package frc.robot.subsystems.climb;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimbInputs {
    public MutableMeasure<Velocity<Velocity<Distance>>> acceleration =
            Units.MetersPerSecondPerSecond.of(0).mutableCopy();
    public ClimbIO.ControlMode controlMode = ClimbIO.ControlMode.POSITION;
    public boolean isBottom = true;
}
