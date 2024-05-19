package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    private final CommandXboxController controller = new CommandXboxController(0);
    private final Intake intake;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        intake = Intake.getInstance(new IntakeIOReal());
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {}

    private void configureButtonBindings() {
        controller.a().whileTrue(intake.intake()).onFalse(intake.stop());
        controller.b().whileTrue(intake.outtake()).onFalse(intake.stop());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
