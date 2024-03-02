package frc.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commandGroups.CommandGroupsConstants;
import frc.robot.lib.Utils;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperIO io;
    private final GripperInputsAutoLogged inputs = GripperIO.inputs;
    private Timer timer = new Timer();

    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(1, 1);

    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0.5, 0.5);
    private final MechanismLigament2d gripperLigament =
            root.append(new MechanismLigament2d("Gripper", 0.3, 0));

    private Gripper(GripperIO io, Supplier<Measure<Distance>> carriageHeight) {
        this.io = io;

        timer.start();
        timer.reset();
    }

    public static Gripper getInstance() {
        return INSTANCE;
    }

    public static void initialize(GripperIO io, Supplier<Measure<Distance>> carriageHeight) {
        INSTANCE = new Gripper(io, carriageHeight);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }


    public Command setRollerPower(double power) {
        return run(() -> io.setRollerMotorPower(power)).withName("set roller power");
    }


    public Command intake() {
        return setRollerPower(
                        GripperConstants.INTAKE_POWER)
                .withName("intake");
    }

    public Command outtake() {
        return setRollerPower(
                        GripperConstants.OUTTAKE_POWER
                       )
                .withName("outtake");
    }



    @Override
    public void periodic() {

        io.updateInputs();
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs(this.getClass().getSimpleName(), inputs);
        }
    }
}
