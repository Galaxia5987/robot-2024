package frc.robot.subsystems.example;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
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
    public void testSetVelocity() {
        shooter.setDefaultCommand(
                shooter.setVelocity(() -> Units.RotationsPerSecond.of(100).mutableCopy()));
        try {
            wait(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        assertTrue(shooter.atSetpoint());
    }

    @Override
    public void close() {}
}
