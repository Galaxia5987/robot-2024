package frc.robot.subsystems.example;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class HoodTest implements AutoCloseable {
    private final Hood hood;
    private final double Delta = 0.5;

    public HoodTest() {
        HoodIO hoodIO = new HoodIOSim();
        Hood.initialize(hoodIO);
        hood = Hood.getInstance();
    }

    @Test
    public void testSetAngle() throws InterruptedException {
        hood.setDefaultCommand(hood.setAngle(() -> Units.Radians.of(0.5).mutableCopy()));
        CommandScheduler.getInstance().run();
        SimHooks.stepTiming(3);
        Assertions.assertTrue(hood.atSetpoint());
    }

    @Test
    public void testSetPower() throws InterruptedException {
        hood.setDefaultCommand(hood.setPower(() -> 0.5));
        CommandScheduler.getInstance().run();
        SimHooks.stepTiming(3);
        Assertions.assertEquals(6, hood.getVoltage().in(Units.Volts), Delta);
    }

    @Override
    public void close() throws Exception {}
}
