package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class SetVelocity extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RotationsPerSecond);

    public SetVelocity(Supplier<Double> velocity) {
        this.velocity.mut_replace(velocity.get(), RotationsPerSecond);
    }

    @Override
    public void initialize() {
        /*
        What to do when the command starts
         */
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        /*
        What to do when the command ends
         */
    }

    @Override
    public boolean isFinished() {
        /*
        Whether the command is finished
         */
        return false;
    }
}
