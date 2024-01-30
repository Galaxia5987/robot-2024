package frc.robot.subsystems.conveyor;

import edu.wpi.first.units.*;

public class ConveyorConstants {

    public static final MutableMeasure<Velocity<Angle>> FEED_VELOCITY =
            Units.RotationsPerSecond.of(10).mutableCopy();
    public static final MutableMeasure<Dimensionless> SETPOINT_TOLERANCE =
            Units.Value.of(0.05).mutableCopy();
}
