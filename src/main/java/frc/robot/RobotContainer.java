package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.example.*;
import frc.robot.swerve.*;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ExampleSubsystemIO exampleSubsystemIO;
        ModuleIO[] moduleIOs = new ModuleIO[4];
        GyroIO gyroIO;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                exampleSubsystemIO = new ExampleSubsystemIOReal();
                for (int i = 0; i < moduleIOs.length; i++) {
                    moduleIOs[i] =
                            new ModuleIOTalonFX(
                                    Ports.SwerveDrive.DRIVE_IDS[i],
                                    Ports.SwerveDrive.ANGLE_IDS[i],
                                    Ports.SwerveDrive.ENCODER_IDS[i],
                                    SwerveConstants.DRIVE_MOTOR_CONFIGS,
                                    SwerveConstants.ANGLE_MOTOR_CONFIGS);
                }
                gyroIO = new GyroIOReal();
                break;
            case SIM:
            case REPLAY:
            default:
                exampleSubsystemIO = new ExampleSubsystemIOSim();
                for (int i = 0; i < moduleIOs.length; i++) {
                    moduleIOs[i] = new ModuleIOSim();
                }
                gyroIO = new GyroIOSim();
                break;
        }
        ExampleSubsystem.initialize(exampleSubsystemIO);
        SwerveDrive.initialize(gyroIO, moduleIOs);

        swerveDrive = SwerveDrive.getInstance();

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

    private void configureDefaultCommands() {}

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
