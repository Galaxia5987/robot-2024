// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.PoseEstimation;
import frc.robot.scoreStates.LocalADStarAK;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private final Field2d field2d = new Field2d();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        if (Robot.isReal()) {
            Constants.CURRENT_MODE = Constants.Mode.REAL;
        } else {
            Constants.CURRENT_MODE = Constants.Mode.SIM;
        }

        // Initialize logger
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.CURRENT_MODE) {
            case REAL:
                LoggedPowerDistribution.getInstance();
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));

                break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        Pathfinding.setPathfinder(new LocalADStarAK());

        SwerveConstants.initConstants(true, Robot.isReal());
        IntakeConstants.initConstants();
        ConveyorConstants.initConstants();
        ElevatorConstants.initConstants();
        GripperConstants.initConstants();
        HoodConstants.initConstants();
        ShooterConstants.initConstants();

        robotContainer = RobotContainer.getInstance();
        compressor.enableDigital();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        PoseEstimation.getInstance()
                .processVisionMeasurements(Constants.VISION_MEASUREMENT_MULTIPLIER);
        ShootingManager.getInstance().updateCommandedState();
        CommandScheduler.getInstance().run();

        Logger.recordOutput(
                "Robot/DistanceToSpeaker", PoseEstimation.getInstance().getDistanceToSpeaker());
        Logger.recordOutput("Robot/IsShooting", ShootingManager.getInstance().isShooting());
        Logger.recordOutput("Robot/ReadyToShoot", ShootingManager.getInstance().readyToShoot());
        field2d.setRobotPose(PoseEstimation.getInstance().getEstimatedPose());
        SmartDashboard.putData("Field", field2d);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        // Make sure command is compiled beforehand, otherwise there will be a delay.
        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the autonomous command
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
