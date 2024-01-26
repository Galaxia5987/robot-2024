package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import frc.robot.subsystems.example.ExampleSubsystemIOReal;
import frc.robot.subsystems.example.ExampleSubsystemIOSim;
import frc.robot.swerve.*;
import frc.robot.vision.PhotonVisionIOReal;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionModule;
import frc.robot.vision.VisionSimIO;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

public class RobotContainer {

    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive;
    private final CommandXboxController xboxController = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        ExampleSubsystemIO exampleSubsystemIO;
        ModuleIO[] moduleIOs = new ModuleIO[4];
        GyroIO gyroIO;
        VisionModule frontLeftVisionModule;
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
                frontLeftVisionModule =
                        new VisionModule(
                                new PhotonVisionIOReal(
                                        new PhotonCamera("Front left camera"),
                                        new Transform3d(0.293, 0.293, 0.2, new Rotation3d()),
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()));
                break;
            case SIM:
            case REPLAY:
            default:
                exampleSubsystemIO = new ExampleSubsystemIOSim();
                for (int i = 0; i < moduleIOs.length; i++) {
                    moduleIOs[i] = new ModuleIOSim();
                }
                gyroIO = new GyroIOSim();
                frontLeftVisionModule =
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera("Front left camera"),
                                        new Transform3d(0.293, 0.293, 0.2, new Rotation3d()),
                                        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                                        SimCameraProperties.PI4_LIFECAM_640_480()));
                break;
        }
        ExampleSubsystem.initialize(exampleSubsystemIO);
        SwerveDrive.initialize(gyroIO, moduleIOs);
        Vision.initialize(frontLeftVisionModule);

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

    private void configureDefaultCommands() {
        swerveDrive.setDefaultCommand(
                swerveDrive.driveCommand(
                        () -> -xboxController.getLeftY(),
                        () -> -xboxController.getLeftX(),
                        () -> -xboxController.getRightX(),
                        0.15,
                        () -> true
                ));
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
