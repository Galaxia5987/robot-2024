package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.example.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.eclipse.jetty.util.thread.TimerScheduler;

import java.util.Timer;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;
    private Intake intake = Intake.getInstance();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ExampleSubsystemIO exampleSubsystemIO;
        IntakeIO intakeIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                exampleSubsystemIO = new ExampleSubsystemIOReal();
                intakeIO = new IntakeIOSim(new PIDController(0,0,0));
                break;
            case SIM:
            case REPLAY:
            default:
                exampleSubsystemIO = new ExampleSubsystemIOSim();
                intakeIO = new IntakeIOSim(new PIDController(ANGLE_KP.get(), ANGLE_KI.get(), ANGLE_KD.get(), 0.02));
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
        intake.setAngle(Units.Degrees.of(0).mutableCopy());
        Units.
    }

    private void configureButtonBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
