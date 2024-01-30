package frc.robot.subsystems.example;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.junit.Test;

public class ShooterTest implements AutoCloseable {
    private final Shooter shooter;

    public ShooterTest() {
        ShooterIO shooterIO = new ShooterIOSim();
        Shooter.initialize(shooterIO);
        shooter = Shooter.getInstance();
    }

    @Test
    public void testSetVelocity() throws InterruptedException {
        shooter.setDefaultCommand(
                shooter.setVelocity(() -> Units.RotationsPerSecond.of(100).mutableCopy()));
        CommandScheduler.getInstance().run();
        SimHooks.stepTiming(3);

        assertTrue(shooter.atSetpoint());
    }

    @Override
    public void close() {}
}
