package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import frc.robot.subsystems.example.ExampleSubsystemIOReal;
import frc.robot.subsystems.example.ExampleSubsystemIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;
    private Intake intake;
    private XboxController xboxController = new XboxController(0);
    private JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ExampleSubsystemIO exampleSubsystemIO;
        IntakeIO intakeIO;
        intake = Intake.getInstance();
        switch (Constants.CURRENT_MODE) {
            case REAL:
                exampleSubsystemIO = new ExampleSubsystemIOReal();
                intakeIO = new IntakeIOSim();
                break;
            case SIM:
            case REPLAY:
            default:
                exampleSubsystemIO = new ExampleSubsystemIOSim();
                intakeIO = new IntakeIOSim();
                break;
        }
        ExampleSubsystem.initialize(exampleSubsystemIO);
        Intake.initialize(intakeIO);

        // Configure the button bindings and default commands
        configureDefaultCommands();
        configureButtonBindings();
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {
        intake.setDefaultCommand(intake.setAngle(Units.Degrees.of(0).mutableCopy()));
    }

    private void configureButtonBindings() {
        a.onTrue(intake.setAngle(Units.Degrees.of(90).mutableCopy()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
