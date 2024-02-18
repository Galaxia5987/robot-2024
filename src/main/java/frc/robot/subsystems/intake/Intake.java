package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE = null;
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = IntakeIO.inputs;
    @AutoLogOutput private final Mechanism2d intakeMechanism = new Mechanism2d(2, 3);
    @AutoLogOutput private final Pose3d robotPose = new Pose3d();
    private final MechanismRoot2d root = intakeMechanism.getRoot("Intake", 1, 1);

    private final MechanismLigament2d intakeLigament =
            root.append(new MechanismLigament2d("IntakeLigmament", 0.21, 0));

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public static Intake getInstance() {
        return INSTANCE;
    }

    public static void initialize(IntakeIO io) {
        INSTANCE = new Intake(io);
    }

    public Command setAngle(MutableMeasure<Angle> angle) {

        return runOnce(
                () -> {
                    inputs.angleSetpoint = angle;
                    io.setAngle(angle);
                });
    }

    public Command setAngle(IntakeConstants.IntakePose intakePose) {
        return setAngle(intakePose.intakePose);
    }

    private Command setRollerSpeed(double speed) {
        return Commands.runOnce(
                () -> {
                    inputs.rollerSpeedSetpoint = speed;
                    io.setRollerSpeed(speed);
                });
    }

    public Command setCenterRollerSpeed(double speed) {
        return Commands.runOnce(
                () -> {
                    inputs.centerRollerSpeedSetpoint = speed;
                    io.setCenterRollerSpeed(speed);
                });
    }

    public Command intake() {
        return Commands.parallel(
                        setAngle(IntakeConstants.IntakePose.DOWN),
                        setRollerSpeed(0.5),
                        setCenterRollerSpeed(0.7))
                .withName("feeding position activated");
    }

    public Command stop() {
        return Commands.parallel(
                        setAngle(IntakeConstants.IntakePose.UP),
                        setRollerSpeed(0),
                        setCenterRollerSpeed(0))
                .withName("stopped");
    }

    public Command reset(Measure<Angle> angle) {
        return runOnce(() -> io.reset(angle));
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs(this.getClass().getSimpleName(), inputs);
        Logger.recordOutput(
                "IntakePose",
                new Pose3d(
                        new Translation3d(0.327, 0, 0.165),
                        new Rotation3d(0, -inputs.currentAngle.in(Units.Radians), 0)));
        intakeLigament.setAngle(inputs.currentAngle.in(Units.Degrees));
    }
}
