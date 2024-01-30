package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
                    HoodConstants.MECHANISM_2D_POSE.getX(),
                    HoodConstants.MECHANISM_2D_POSE.getY());
    private final MechanismLigament2d hood =
            root.append(
                    new MechanismLigament2d(
                            "Hood", HoodConstants.HOOD_LENGTH.in(Units.Meters), 45));

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

    public MutableMeasure<Voltage> getVoltage() {
        return inputs.voltage;
    }

    public boolean atSetpoint() {
        return inputs.angle.isNear(
                inputs.angleSetpoint, HoodConstants.POSITION_TOLERANCE.in(Units.Value));
    }

    public Command setAngle(Supplier<MutableMeasure<Angle>> angle) {
        return runOnce(() -> inputs.controlMode = HoodIO.Mode.ANGLE)
                .andThen(run(() -> io.setAngle(angle.get())))
                .withName("Set hood angle");
    }

    public Command setPower(Supplier<Double> power) {
        return runOnce(() -> inputs.controlMode = HoodIO.Mode.POWER)
                .andThen(run(() -> io.setPower(power.get())))
                .withName("Set hood power");
    }

    public Command updateInternalEncoder() {
        return runOnce(io::updateInternalEncoder).withName("Update hood internal encoder");
    }

    private Pose3d getPose3d(double angle) {
        return new Pose3d(
                HoodConstants.ROOT_POSITION.getX(), 0.0, HoodConstants.ROOT_POSITION.getY(), new Rotation3d(0.0, -angle, 0.0));
    }

    /** Updates the state of the hood. */
    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Hood", inputs);

        hood.setAngle(inputs.angle.in(Units.Degrees));

        Logger.recordOutput("HoodPose", getPose3d(inputs.angle.in(Units.Degrees)));
        SmartDashboard.putData("HoodMech", mechanism2d);
    }
}
