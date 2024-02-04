package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakePose;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE = null;
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = IntakeIO.inputs;
    @AutoLogOutput private final Mechanism2d intakeMechanism = new Mechanism2d(2, 3);
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
        inputs.angleSetPoint = angle;
        return runOnce(() -> io.setAngle(Units.Degrees.of( Math.IEEEremainder(angle.in(Units.Degrees), 180)).mutableCopy()));
    }

    public Command setAngle(IntakePose intakePose) {
        return setAngle(intakePose.intakePose);
    }

    public Command setRollerSpeed(double speed) {
        inputs.rollerSpeedSetPoint = Units.RotationsPerSecond.of(speed).mutableCopy();
        return runOnce(() -> io.setRollerSpeed(speed));
    }

    public Command setCenterRollerSpeed(double speed) {
        inputs.centerRollerSpeedSetPoint = Units.RotationsPerSecond.of(speed).mutableCopy();
        return runOnce(() -> io.setCenterRollerSpeed(speed));
    }

    public Command intake() {
        return setAngle(IntakePose.DOWN)
                .alongWith(setRollerSpeed(0.5).andThen(setCenterRollerSpeed(0.5)))
                .withName("feeding position activated");
    }

    public Command stop() {
        return setAngle(IntakePose.UP)
                .alongWith(setRollerSpeed(0).andThen(setCenterRollerSpeed(0)))
                .withName("stopped");
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
