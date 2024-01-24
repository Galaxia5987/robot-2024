package frc.robot.subsystems.hood;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    private static Hood INSTANCE = null;
    private final HoodInputsAutoLogged inputs = HoodIO.inputs;
    private final HoodIO io;
    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(3, 3);
    private final MechanismRoot2d root =
            mechanism2d.getRoot(
                    "Hood",
                    HoodConstants.mechanism2dPose.getX(),
                    HoodConstants.mechanism2dPose.getY());
    private final MechanismLigament2d hood =
            root.append(new MechanismLigament2d("Hood", HoodConstants.HoodLength, 45));

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
        return inputs.angle.isNear(inputs.angleSetpoint, HoodConstants.positionTolerance);
    }

    public Command setAngle(Supplier<MutableMeasure<Angle>> angle) {
        return run(() -> {
            inputs.angleSetpoint.mut_replace(angle.get());
            io.setAngle(inputs.angleSetpoint);
        });
    }

    public Command setPower(Supplier<Double> power) {
        return run(() -> {
            inputs.powerSetpoint = power.get();
            io.setPower(inputs.powerSetpoint);
        });
    }

    public Command resetAbsoluteEncoder() {
        return Commands.sequence(
                setPower(() -> HoodConstants.resetPower),
                runOnce(io::resetAbsoluteEncoder),
                setPower(() -> 0.0));
    }

    public Command updateInternalEncoder() {
        return runOnce(io::updateInternalEncoder);
    }

    /** Updates the state of the hood. */
    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Hood", inputs);
    }
}
