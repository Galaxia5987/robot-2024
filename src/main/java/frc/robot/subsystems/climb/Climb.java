package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.MECHANISM_HEIGHT;
import static frc.robot.subsystems.climb.ClimbConstants.MECHANISM_WIDTH;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

    private static Climb INSTANCE;

    private final ClimbInputsAutoLogged inputs = ClimbIO.inputs;
    private final ClimbIO io;
    private final Timer timer = new Timer();

    @AutoLogOutput
    private final Mechanism2d mechanism2d = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);

    private final MechanismRoot2d root = mechanism2d.getRoot("Elevator", 0, 0);
    private final MechanismLigament2d elevator =
            root.append(new MechanismLigament2d("Elevator", 0, 90));

    private Climb(ClimbIO io) {
        this.io = io;

        timer.start();
        timer.reset();
    }

    public static Climb getInstance() {
        return INSTANCE;
    }

    public static void initialize(ClimbIO io) {
        INSTANCE = new Climb(io);
    }

    public Command manualClimb(DoubleSupplier power) {
        return Commands.parallel(run(() -> io.setPower(power.getAsDouble())));
    }

    public Command manualReset() {
        return Commands.runOnce(io::manualReset);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs(this.getClass().getSimpleName(), inputs);
        }
    }
}
