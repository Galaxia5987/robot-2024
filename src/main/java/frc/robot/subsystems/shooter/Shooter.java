package frc.robot.subsystems.shooter;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Utils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static Shooter INSTANCE = null;
    private final RollerInputsAutoLogged topRollerInputs = ShooterIO.topRollerInputs;
    private final RollerInputsAutoLogged bottomRollerInputs = ShooterIO.bottomRollerInputs;
    private final ShooterIO io;
    private final Timer timer = new Timer();

    private final String SUBSYSTEM_NAME = this.getClass().getSimpleName();

    /**
     * Constructor for Shooter subsystem.
     *
     * @param io IO of the subsystem.
     */
    private Shooter(ShooterIO io) {
        this.io = io;

        timer.start();
        timer.reset();
    }

    /**
     * Gets the single instance of Shooter.
     *
     * @return The single instance of Shooter.
     */
    public static Shooter getInstance() {
        return INSTANCE;
    }

    public static void initialize(ShooterIO io) {
        INSTANCE = new Shooter(io);
    }

    public Command setVelocity(
            MutableMeasure<Velocity<Angle>> topVelocity,
            MutableMeasure<Velocity<Angle>> bottomVelocity) {
        return run(() -> {
                    topRollerInputs.velocitySetpoint.mut_replace(topVelocity);
                    io.setTopVelocity(topVelocity);
                    bottomRollerInputs.velocitySetpoint.mut_replace(bottomVelocity);
                    io.setBottomVelocity(bottomVelocity);
                })
                .withName("Set Shooter Velocity");
    }

    public Command setVelocity(MutableMeasure<Velocity<Angle>> velocity) {
        return setVelocity(velocity, velocity);
    }

    public Command stop() {
        return run(() -> {
                    topRollerInputs.velocitySetpoint.mut_replace(ShooterConstants.STOP_POWER);
                    bottomRollerInputs.velocitySetpoint.mut_replace(ShooterConstants.STOP_POWER);
                    io.stop();
                })
                .withName("Stop Shooter");
    }

    @AutoLogOutput
    public boolean atSetpoint() {
        return Utils.epsilonEquals(topRollerInputs.velocity.in(Units.RotationsPerSecond),
                topRollerInputs.velocitySetpoint.in(Units.RotationsPerSecond),
                1) &&
                Utils.epsilonEquals(bottomRollerInputs.velocity.in(Units.RotationsPerSecond),
                        bottomRollerInputs.velocitySetpoint.in(Units.RotationsPerSecond),
                        1);
    }

    /** Updates the state of the shooter. */
    @Override
    public void periodic() {
        io.updateInputs();
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs(SUBSYSTEM_NAME + "/TopRoller", topRollerInputs);
            Logger.processInputs(SUBSYSTEM_NAME + "/BottomRoller", bottomRollerInputs);
        }
    }
}
