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

    private ControlMode controlMode = ControlMode.POSITION;

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

    public MutableMeasure<Angle> getAngle() {
        return inputs.angle;
    }

    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    public Command setAngle(Supplier<Rotation2d> angle){
        controlMode = ControlMode.POSITION;
        return runOnce(() -> inputs.angleSetpoint.mut_replace(angle.get().getRadians(), Radians));
    }

    public Command setPower(Supplier<Double> power){
        controlMode = ControlMode.POWER;
        return runOnce(() -> inputs.powerSetpoint = power.get());
    }

    public Command getResetAbsoluteEncoderCommand(){
        setPower(() -> -0.3);
        return runOnce(() -> io.resetAbsoluteEncoder());
    }

    public Command getUpdateInternalEncoderCommand(){
        return runOnce(() -> io.updateInternalEncoder());
    }

    /** Updates the state of the hood. */
    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Hood", inputs);

        if (controlMode == ControlMode.POSITION) {
            io.setAngle(inputs.angleSetpoint);
        } else {
            io.setPower(inputs.powerSetpoint);
        }
    }

    public enum ControlMode {
        POSITION,
        POWER
    }
}
