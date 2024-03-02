package frc.robot.commandGroups;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.PoseEstimation;
import frc.robot.lib.math.interpolation.InterpolatingDouble;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class CommandGroups {
    private static CommandGroups INSTANCE;
    private final Intake intake;
    private final Gripper gripper;
    private final Climb elevator;
    private final Shooter shooter;
    private final Hood hood;
    private final Conveyor conveyor;
    private final LEDs leds;
    private final SwerveDrive swerveDrive;
    private boolean override;

    private CommandGroups() {
        leds = new LEDs(0, 60);
        intake = Intake.getInstance();
        gripper = Gripper.getInstance();
        elevator = Climb.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        conveyor = Conveyor.getInstance();
        swerveDrive = SwerveDrive.getInstance();

        leds.setPrimary(Color.kAliceBlue);
        leds.setSecondary(Color.kOrangeRed);
        override = false;
    }

    public static CommandGroups getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CommandGroups();
        }
        return INSTANCE;
    }

    public void toggleOverride() {
        override = !override;
    }

    public Command feed() {
        return conveyor.feed()
                .alongWith(
                        Commands.waitUntil(conveyor::readyToFeed)
                                .andThen(gripper.setRollerPower(GripperConstants.OUTTAKE_POWER)))
                .withName("feed");
    }

    public Command feedWithWait(BooleanSupplier otherReady) {
        return Commands.waitUntil(otherReady)
                .andThen(gripper.setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.4))
                .withName("feedWithWait");
    }

    public Command feedShooter(BooleanSupplier isForceShooting) {
        return feedWithWait(
                        () ->
                                ShootingManager.getInstance().readyToShoot()
                                        || isForceShooting.getAsBoolean())
                .withName("feedShooter");
    }

    public Command intake() {
        return Commands.parallel(
                        intake.intake(),
                        gripper.setRollerPower(0.3)
                                .until(gripper::hasNote)
                                .andThen(gripper.setRollerPower(0))
                                .alongWith(leds.solidSecondary(1, 60)))
                .withName("intake");
    }

    public Command setShooter() {
        return Commands.repeatingSequence(
                shooter.setVelocity(ShootingManager.getInstance().getShooterCommandedVelocity())
                        .until(shooter::atSetpoint));
    }

    public Command setScoringSystems() {
        return Commands.parallel(setHood(), setShooter());
    }

    public Command setHood() {
        var distanceToSpeaker =
                new InterpolatingDouble(PoseEstimation.getInstance().getDistanceToSpeaker());
        return Commands.repeatingSequence(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle())
                        .until(hood::atSetpoint));
    }

    public Command outtakeGripper() {
        return gripper.outtake().withName("outtakeGripper");
    }

    public Command outtakeShooter() {
        return Commands.parallel(feed(), shooter.setVelocity(ShooterConstants.OUTTAKE_POWER))
                .withName("outtakeShooter");
    }

    public Command shootAndConvey(
            MutableMeasure<Velocity<Angle>> topVelocity,
            MutableMeasure<Velocity<Angle>> bottomVelocity) {
        return shooter.setVelocity(topVelocity, bottomVelocity)
                .alongWith(conveyor.setVelocity(bottomVelocity));
    }

    public Command shootAndConvey(MutableMeasure<Velocity<Angle>> velocity) {
        return shooter.setVelocity(velocity).alongWith(conveyor.setVelocity(velocity));
    }

    public Command intakeBit() {
        return Commands.sequence(intake().withTimeout(3), intake.stop());
    }

    public Command shooterBit() {
        return shootToAmp()
                .andThen(
                        Commands.waitSeconds(3),
                        shooter.stop(),
                        conveyor.stop(),
                        hood.setAngle(Units.Degrees.of(114).mutableCopy()));
    }

    public Command shootAndIntake() {
        return Commands.parallel(gripper.setRollerPower(0.7), intake.intake());
    }

    public Command shootToAmp() {
        return shooter.setVelocity(
                        ShooterConstants.TOP_AMP_VELOCITY, ShooterConstants.BOTTOM_AMP_VELOCITY)
                .alongWith(hood.setAngle(HoodConstants.AMP_ANGLE))
                .until(shooter::atSetpoint)
                .andThen(gripper.setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.5))
                .andThen(gripper.setRollerPower(0))
                .alongWith(conveyor.setVelocity(ConveyorConstants.AMP_VELOCITY));
    }

    public Command shootToSpeaker(CommandXboxController driveController) {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                shootAndConvey(ShootingManager.getInstance().getShooterCommandedVelocity()),
                swerveDrive.driveAndAdjust(
                        ShootingManager.getInstance().getSwerveCommandedAngle(),
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        0.1));
    }

    public Command shootToSpeaker() {
        return Commands.parallel(
                hood.setAngle(ShootingManager.getInstance().getHoodCommandedAngle()),
                shootAndConvey(ShootingManager.getInstance().getShooterCommandedVelocity()));
    }

    public Command stopShooting() {
        return Commands.parallel(
                hood.setAngle(Units.Degrees.of(114).mutableCopy()),
                shooter.setVelocity(Units.RotationsPerSecond.zero().mutableCopy()),
                conveyor.stop(),
                gripper.setRollerPower(0));
    }

    public Command allBits() {
        return Commands.sequence(intakeBit(), shooterBit().withTimeout(3));
    }
}
