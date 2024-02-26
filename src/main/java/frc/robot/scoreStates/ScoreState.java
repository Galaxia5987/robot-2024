package frc.robot.scoreStates;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;

public interface ScoreState {
    static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    default Command calculateTargets() {
        return Commands.none();
    }

    default Command prepareSubsytems() {
        return Commands.none();
    }

    Command score(Optional<CommandXboxController> driveController, boolean isAuto);

    default MutableMeasure<Angle> getSwerveAngle() {
        return Units.Degrees.of(0).mutableCopy();
    }
}
