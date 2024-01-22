package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private static Elevator INSTANCE;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
    private final ElevatorIO io;
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


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        Logger.recordOutput("current height", getCurrentHeight());
    }
}
