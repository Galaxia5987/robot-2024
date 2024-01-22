package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private static Elevator INSTANCE;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
    private final ElevatorIO io;
    private final Mechanism2d mechanism2d = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);
    private final MechanismRoot2d root = mechanism2d.getRoot("Elevator", 0, 0);
    private final MechanismLigament2d elevator = root.append(new MechanismLigament2d("Elevator", 0, 0));
    private Command lastCommand = null;
    private Command currentCommand = null;
    private boolean changedToDefaultCommand = false;

    private Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void setPower(double power) {
        io.setPower(power);
    }

    public void setHeight(MutableMeasure<Distance> height) {
        io.setHeight(height);
    }

    public MutableMeasure<Distance> getHeight() {
        return inputs.height;
    }

    public MutableMeasure<Distance> getHeightSetpoint() {
        return inputs.heightSetpoint;
    }

    public boolean isBottom(){
        return inputs.isBottom;
    }

    public boolean isTop(){
        return inputs.isTop;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getClass().getSimpleName(),  inputs);
        currentCommand = getCurrentCommand();
    }
}
