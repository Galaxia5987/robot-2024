package frc.robot.subsystems.hood.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import org.apache.commons.lang3.mutable.Mutable;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;

public class SetAngle extends Command {
    private final Hood hood = Hood.getInstance();
    private MutableMeasure<Angle> angle;

    public SetAngle(Supplier<Rotation2d> angle) {
        this.angle.mut_replace(angle.get().getRadians(), Radians);
    }

    @Override
    public void initialize() {
        /*
        What to do when the command starts
         */
    }

    @Override
    public void execute() {
        hood.setAngle(angle);
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
