package frc.robot.subsystems.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakePose;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE = null;
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = IntakeIO.inputs;
    private final Mechanism2d intakeMechanism = new Mechanism2d(2, 3);
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
        return runOnce(() -> io.setAngle(angle));
    }

    public Command setAngle(IntakePose intakePose) {
        return setAngle(intakePose.intakePose);
    }

    public Command setRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        return runOnce(() -> io.setRollerSpeed(speed));
    }

    public Command setCenterRollerSpeed(MutableMeasure<Velocity<Angle>> speed) {
        return runOnce(() -> io.setCenterRollerSpeed(speed));
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs(this.getClass().getName(), inputs);
        intakeLigament.setAngle(inputs.currentAngle.in(Units.Degrees));
    }
}
