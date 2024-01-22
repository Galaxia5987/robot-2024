package frc.robot.subsystems.gripper;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperInputs inputs = new GripperInputs();
    private final GripperIO io;
    private final Mechanism2d mechanism2d = new Mechanism2d(
            0, 0
    );
    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0, 0);
    private final MechanismLigament2d shoulder = root.append(
            new MechanismLigament2d("Gripper", 0, 0)
    );
    private final GripperInputsAutoLogged inputs = new GripperInputsAutoLogged();


    public Gripper(GripperIO io) {
        this.io = io;
    }


    public void setSpeedMotorPower(double power) {
        io.setPower(power);
    }

    public void setAngle(MutableMeasure<Angle> angle) {
        io.setAngle(angle);
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

    public MutableMeasure<Voltage> spinMotorVoltage(){
        return inputs.spinMotorVoltage;
    }
    public MutableMeasure<Voltage> angleMotorVoltage(){
        return inputs.angleMotorVoltage;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        Logger.recordOutput("current angle", getCurrentAngle());
    }
}


