package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Utils;
import java.util.function.BooleanSupplier;
import lombok.Setter;
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
                    HoodConstants.MECHANISM_2D_POSE.getX(),
                    HoodConstants.MECHANISM_2D_POSE.getY());
    private final MechanismLigament2d hood =
            root.append(
                    new MechanismLigament2d(
                            "Hood", HoodConstants.HOOD_LENGTH.in(Units.Meters), 45));
    private final Timer timer = new Timer();
    private final Timer encoderTimer = new Timer();

    @Setter private double chassisCompensationTorque;

    /**
     * Constructor for Hood subsystem.
     *
     * @param io IO of the subsystem.
     */
    private Hood(HoodIO io) {
        this.io = io;

        timer.start();
        timer.reset();

        encoderTimer.start();
        encoderTimer.reset();
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
        return inputs.internalAngle;
    }

    @AutoLogOutput
    public boolean atSetpoint() {
        return Utils.epsilonEquals(
                inputs.absoluteEncoderAngle.in(Units.Degrees),
                inputs.angleSetpoint.in(Units.Degrees),
                DriverStation.isAutonomous() ? 1.0 : 0.5);
    }

    public boolean atSetpointFast() {
        return Utils.epsilonEquals(
                inputs.absoluteEncoderAngle.in(Units.Degrees),
                inputs.angleSetpoint.in(Units.Degrees),
                2.0);
    }

    public Command setAngle(MutableMeasure<Angle> angle) {
        return setAngle(angle, () -> false);
    }

    public Command setAngle(MutableMeasure<Angle> angle, BooleanSupplier useChassisCompensation) {
        return run(() -> {
                    inputs.angleSetpoint.mut_replace(angle);
                    if (useChassisCompensation.getAsBoolean()) {
                        io.setAngle(angle, chassisCompensationTorque);
                    } else {
                        io.setAngle(angle);
                    }
                })
                .withName("Set hood angle");
    }

    @AutoLogOutput(key = "Hood/Pose")
    private Pose3d getPose3d() {
        return new Pose3d(
                HoodConstants.ROOT_POSITION,
                new Rotation3d(
                        0.0,
                        getAngle().plus(HoodConstants.SIMULATION_OFFSET).in(Units.Radians),
                        0.0));
    }

    /** Updates the state of the hood. */
    @Override
    public void periodic() {
        io.updateInputs();
        if (encoderTimer.advanceIfElapsed(0.5)) {
            io.updateInternalEncoder();
        }
        if (timer.advanceIfElapsed(0.0)) {
            Logger.processInputs("Hood", inputs);
        }

        hood.setAngle(inputs.internalAngle.in(Units.Degrees));
    }
}
