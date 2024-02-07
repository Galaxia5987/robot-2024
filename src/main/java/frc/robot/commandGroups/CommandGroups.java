package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.scoreStates.ScoreState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

public class CommandGroups {
    private static CommandGroups INSTANCE = null;
    private Intake intake;
    private Gripper gripper;
    private Elevator elevator;

    public CommandGroups() {
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        elevator = Elevator.getInstance();
    }

    public static CommandGroups getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new CommandGroups();
        }
        return INSTANCE;
    }
}
