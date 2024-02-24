package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.MECHANISM_HEIGHT;
import static frc.robot.subsystems.elevator.ElevatorConstants.MECHANISM_WIDTH;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private static Elevator INSTANCE;

    private final ElevatorInputsAutoLogged inputs = ElevatorIO.inputs;
    private final ElevatorIO io;
    private final Timer timer = new Timer();

    @AutoLogOutput
    private final Mechanism2d mechanism2d = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);

    @AutoLogOutput private Pose3d elevatorPose = new Pose3d(0, 0, 0, new Rotation3d());
    @AutoLogOutput private Pose3d carriagePose = new Pose3d(0, 0, 0, new Rotation3d());

    private final MechanismRoot2d root = mechanism2d.getRoot("Elevator", 0, 0);
    private final MechanismLigament2d elevator =
            root.append(new MechanismLigament2d("Elevator", 0, 90));

    private Elevator(ElevatorIO io) {
        this.io = io;

        timer.start();
        timer.reset();
    }

    public static Elevator getInstance() {
        return INSTANCE;
    }

    public static void initialize(ElevatorIO io) {
        INSTANCE = new Elevator(io);
    }

    public MutableMeasure<Distance> getCurrentHeight() {
        return inputs.hooksHeight;
    }

    public MutableMeasure<Distance> getCarriageHeight() {
        return inputs.carriageHeight;
    }

    public MutableMeasure<Distance> getHeightSetpoint() {
        return inputs.heightSetpoint;
    }

    public boolean atHeightSetpoint() {
        return inputs.heightSetpoint.isNear(
                inputs.hooksHeight, ElevatorConstants.HEIGHT_TOLERANCE.in(Units.Value));
    }

    public boolean stopperAtSetpoint() {
        return inputs.stopperAngle.isNear(
                inputs.stopperSetpoint, ElevatorConstants.STOPPER_TOLERANCE.in(Units.Value));
    }

    public Command lock() {
        return Commands.runOnce(io::closeStopper);
    }

    public Command unlock() {
        return Commands.runOnce(io::openStopper);
    }

    public Command setHeight(MutableMeasure<Distance> height) {
        return Commands.sequence(
                        runOnce(() -> inputs.heightSetpoint.mut_replace(height)),
                        run(() -> io.setHeight(height)).until(this::atHeightSetpoint))
                .withName("set height");
    }

    public Command manualElevator(DoubleSupplier power) {
        return Commands.parallel(run(() -> io.setPower(power.getAsDouble())));
    }

    public Command manualReset() {
        return Commands.runOnce(io::manualReset);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        elevator.setLength(getCurrentHeight().in(Units.Meters));
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs(this.getClass().getSimpleName(), inputs);
        }

        Logger.recordOutput(
                "elevatorPose",
                new Pose3d(
                        new Translation3d(0, 0, inputs.carriageHeight.in(Units.Meters)),
                        new Rotation3d(0, 0, 0)));
        Logger.recordOutput(
                "carriagePose",
                new Pose3d(
                        new Translation3d(0, 0, inputs.hooksHeight.in(Units.Meters)),
                        new Rotation3d(0, 0, 0)));
    }
}
