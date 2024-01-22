package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.MECHANISM_HEIGHT;
import static frc.robot.subsystems.elevator.ElevatorConstants.MECHANISM_WIDTH;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private static Elevator INSTANCE;

    private final ElevatorInputsAutoLogged inputs = ElevatorIO.inputs;
    private final ElevatorIO io;

    @AutoLogOutput
    private final Mechanism2d mechanism2d = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);

    private final MechanismRoot2d root = mechanism2d.getRoot("Elevator", 0, 0);
    private final MechanismLigament2d elevator =
            root.append(new MechanismLigament2d("Elevator", 0, 0));

    private Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void setPower(double power) {
        io.setPower(power);
        inputs.controlMode = ElevatorIO.ControlMode.PERCENT_OUTPUT;
    }

    public void setHeight(MutableMeasure<Distance> height) {
        io.setHeight(height);
        inputs.controlMode = ElevatorIO.ControlMode.POSITION;
    }

    public MutableMeasure<Distance> getCurrentHeight() {
        return inputs.currentHeight;
    }

    public MutableMeasure<Distance> getHeightSetpoint() {
        return inputs.heightSetpoint;
    }

    public Command setHeight(double height) {
        return run(() -> this.setHeight(height));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        elevator.setLength(getCurrentHeight().in(Units.Meters));
        Logger.processInputs(this.getClass().getSimpleName(), inputs);
    }
}
