package frc.robot.subsystems.gripper;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperIO io;
    private final GripperInputsAutoLogged inputs = GripperIO.inputs;

    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(0, 0);

    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0, 0);
    private final MechanismLigament2d gripperLigament =
            root.append(new MechanismLigament2d("Gripper", 0, 0));

    private Gripper(GripperIO io) {
        this.io = io;
    }

    public static Gripper getInstance() {
        return INSTANCE;
    }

    public static void initialize(GripperIO io) {
        INSTANCE = new Gripper(io);
    }

    public MutableMeasure<Angle> getCurrentAngle() {
        return inputs.currentAngle;
    }

    public MutableMeasure<Angle> getAngleSetpoint() {
        return inputs.angleSetpoint;
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public MutableMeasure<Voltage> getRollerMotorVoltage() {
        return inputs.rollerMotorVoltage;
    }

    public MutableMeasure<Voltage> getAngleMotorVoltage() {
        return inputs.angleMotorVoltage;
    }

    public Command setRollerPower(double power) {
        return run(() -> io.setRollerMotorPower(power)).withName("set roller power");
    }

    public Command intake() {
        return setWristPosition(GripperConstants.INTAKE_ANGLE)
                .alongWith(setRollerPower(GripperConstants.INTAKE_POWER))
                .withName("intake");
    }

    public Command outtake() {
        return setWristPosition(GripperConstants.OUTTAKE_ANGLE)
                .alongWith(setRollerPower(GripperConstants.OUTTAKE_POWER))
                .withName("outtake");
    }

    public Command setWristPosition(MutableMeasure<Angle> angle) {
        return runOnce(() -> io.setAngle(angle)).withName("set wrist position");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        gripperLigament.setLength(getCurrentAngle().in(Units.Radians));
    }
}
