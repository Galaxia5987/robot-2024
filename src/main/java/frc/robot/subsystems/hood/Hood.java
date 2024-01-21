package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;

public class Hood extends SubsystemBase {
    private static Hood INSTANCE = null;
    private final HoodInputsAutoLogged inputs = HoodIO.inputs;
    private final HoodIO io;

    /**
     * Constructor for Hood subsystem.
     *
     * @param io IO of the subsystem.
     */
    private Hood(HoodIO io) {
        this.io = io;
    }

    /**
     * Gets the single instance of hoodSubsystem.
     *
     * @return The single instance of hoodSubsystem.
     */
    public static Hood getInstance() {
        return INSTANCE;
    }

    public static void initialize(HoodIO io) {
        INSTANCE = new Hood(io);
    }

    /**
     * Sets the position of the hood.
     *
     * @param angle The angle of the hood to set.
     */
    private void setAngle(MutableMeasure<Angle> angle) {

    }

    public MutableMeasure<Angle> getAngle() {
        return inputs.angle;
    }

    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    public Command setAngle(Supplier<Rotation2d> angle){
        return run(() -> inputs.angleSetpoint.mut_replace(angle.get().getRadians(), Radians));
    }

    public Command getResetAbsoluteEncoderCommand(){
        return run(() -> io.resetAbsoluteEncoder());
    }

    public Command getUpdateInternalEncoderCommand(){
        return run(() -> io.updateInternalEncoder());
    }

    /** Updates the state of the hood. */
    @Override
    public void periodic() {
        io.setAngle(inputs.angleSetpoint);

        io.updateInputs();
        Logger.processInputs("Hood", inputs);
    }
}
