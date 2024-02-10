package frc.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commandGroups.CommandGroupsConstants;
import java.util.function.Supplier;
import lib.Utils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperIO io;
    private final GripperInputsAutoLogged inputs = GripperIO.inputs;
    private Measure<Distance> gripperHeight = Units.Meters.zero();
    private Measure<Distance> carriageHeight = Units.Meters.zero();

    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(1, 1);
    @AutoLogOutput private Pose3d gripperPose = new Pose3d();

    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0.5, 0.5);
    private final MechanismLigament2d gripperLigament =
            root.append(new MechanismLigament2d("Gripper", 0.3, 0));

    private Gripper(GripperIO io, Supplier<Measure<Distance>> carriageHeight) {
        this.io = io;
        this.carriageHeight = carriageHeight.get();
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
                inputs.angleSetpoint, GripperConstants.THRESHOLD.in(Units.Percent));
    }

    public boolean isReversed() {
        return Utils.normalize(inputs.currentAngle.in(Units.Radians))
                > CommandGroupsConstants.WRIST_MIN_BACKWARDS_ANGLE.in(Units.Radians);
    }

    public boolean isGripperNearFoldedPosition() {
        return new Translation2d(
                                        GripperConstants.GRIPPER_LENGTH.in(Units.Meters),
                                        new Rotation2d(inputs.currentAngle))
                                .getY()
                        + gripperHeight.in(Units.Meters)
                < GripperConstants.GRIPPER_OUTTAKE_MIN_HEIGHT.in(Units.Meters);
    }

    public Command setRollerPower(double power) {
        var command =
                Commands.runOnce(() -> io.setRollerMotorPower(power))
                        .withName("set roller power")
                        .finallyDo(() -> setRollerPower(0));
        command.addRequirements(this);
        return command;
    }

    public Command intake() {
        return setWristPosition(GripperConstants.INTAKE_ANGLE)
                .alongWith(setRollerPower(GripperConstants.INTAKE_POWER))
                .withName("intake");
    }

    public Command outtake() {
        return setWristPosition(GripperConstants.OUTTAKE_ANGLE)
                .alongWith(setRollerPower(GripperConstants.OUTTAKE_POWER))
                .withName("outtake");
    }

    public Command setWristPosition(MutableMeasure<Angle> angle) {
        return runOnce(() -> io.setAngle(angle))
                .until(this::atSetpoint)
                .withName("set wrist position");
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
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        gripperLigament.setAngle(new Rotation2d(inputs.currentAngle));
    }
}
