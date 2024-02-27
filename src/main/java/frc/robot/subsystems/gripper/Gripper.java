package frc.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commandGroups.CommandGroupsConstants;
import frc.robot.lib.Utils;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperIO io;
    private final GripperInputsAutoLogged inputs = GripperIO.inputs;
    private Measure<Distance> gripperHeight = Units.Meters.zero();
    private Measure<Distance> carriageHeight = Units.Meters.zero();
    private Timer timer = new Timer();

    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(1, 1);
    @AutoLogOutput private Pose3d gripperPose = new Pose3d();

    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0.5, 0.5);
    private final MechanismLigament2d gripperLigament =
            root.append(new MechanismLigament2d("Gripper", 0.3, 0));

    private Gripper(GripperIO io, Supplier<Measure<Distance>> carriageHeight) {
        this.io = io;
        this.carriageHeight = carriageHeight.get();

        timer.start();
        timer.reset();
    }

    public static Gripper getInstance() {
        return INSTANCE;
    }

    public static void initialize(GripperIO io, Supplier<Measure<Distance>> carriageHeight) {
        INSTANCE = new Gripper(io, carriageHeight);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public boolean atSetpoint() {
        return inputs.currentAngle.isNear(
                inputs.angleSetpoint, GripperConstants.TOLERANCE.in(Units.Percent));
    }

    public boolean isReversed() {
        return Utils.normalize(inputs.currentAngle.in(Units.Radians))
                > CommandGroupsConstants.WRIST_MIN_BACKWARDS_ANGLE.in(Units.Radians);
    }

    public Command setRollerPower(double power) {
        return run(() -> io.setRollerMotorPower(power)).withName("set roller power");
    }

    public boolean isGripperNearFoldedPosition() {
        return new Translation2d(
                                        GripperConstants.GRIPPER_LENGTH.in(Units.Meters),
                                        new Rotation2d(inputs.currentAngle))
                                .getY()
                        + gripperHeight.in(Units.Meters)
                < GripperConstants.GRIPPER_OUTTAKE_MIN_HEIGHT.in(Units.Meters);
    }

    public Command setRollerAndWrist(double rollerPower, MutableMeasure<Angle> wristAngle) {
        return setRollerPower(rollerPower).raceWith(setWristPosition(wristAngle));
    }

    public Command intake() {
        return setRollerAndWrist(
                        GripperConstants.INTAKE_POWER, GripperConstants.INTAKE_ANGLE.mutableCopy())
                .withName("intake");
    }

    public Command outtake() {
        return setRollerAndWrist(
                        GripperConstants.OUTTAKE_POWER,
                        GripperConstants.OUTTAKE_ANGLE.mutableCopy())
                .withName("outtake");
    }

    public Command setWristPosition(MutableMeasure<Angle> angle) {
        return Commands.runOnce(() -> io.setAngle(angle)).withName("set wrist position");
    }

    public Command scoreTrap() {
        return Commands.sequence(
                setWristPosition(GripperConstants.WRIST_TRAP_ANGLE),
                setRollerPower(GripperConstants.TRAP_POWER));
    }

    public Command setWristPower(DoubleSupplier power) {
        return run(() -> io.setAngleMotorPower(power.getAsDouble())).withName("set wrist power");
    }

    @Override
    public void periodic() {
        gripperHeight = carriageHeight.plus(GripperConstants.GRIPPER_POSITION_z);
        gripperPose =
                new Pose3d(
                        new Translation3d(
                                GripperConstants.GRIPPER_POSITION_X,
                                GripperConstants.GRIPPER_POSITION_Y,
                                gripperHeight),
                        new Rotation3d(0, -inputs.currentAngle.in(Units.Radians), 0));

        io.updateInputs();
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs(this.getClass().getSimpleName(), inputs);
        }

        gripperLigament.setAngle(new Rotation2d(inputs.currentAngle));
    }
}
