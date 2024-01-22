package frc.robot.subsystems.gripper;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperIO io;
    public GripperIO.ControlMode controlMode;
    private final GripperInputsAutoLogged inputs = GripperIO.inputs;

    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(0, 0);

    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0, 0);
    private final MechanismLigament2d gripper =
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

    public void setSpeedMotorPower(double power) {
        io.setSpeedMotorPower(power);
        controlMode = GripperIO.ControlMode.PERCENT_OUTPUT;
    }

    public void setAngleMotorPower(double power) {
        io.setAngleMotorPower(power);
        controlMode = GripperIO.ControlMode.PERCENT_OUTPUT;
    }

    public void setAngle(MutableMeasure<Angle> angle) {
        io.setAngle(angle);
        controlMode = GripperIO.ControlMode.POSITION;
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        Logger.recordOutput("current angle", getCurrentAngle());
    }
}
