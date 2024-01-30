package frc.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperIO io;
    private final GripperInputsAutoLogged inputs = GripperIO.inputs;
    private final ElevatorInputsAutoLogged elevatorInputs = ElevatorIO.inputs;

    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(0, 0);
    @AutoLogOutput private Pose3d gripperPose = new Pose3d(0, 0, 0, new Rotation3d());
    @AutoLogOutput private MutableMeasure<Distance> gripperHeight = null;

    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0, 0);
    private final MechanismLigament2d gripperLigament =
            root.append(new MechanismLigament2d("Gripper", 0, 0));

    private Gripper(GripperIO io) {
        this.io = io;
    }

    public static Gripper getInstance() {
        return INSTANCE;
    }

    public static void initialize(GripperIO io) {
        INSTANCE = new Gripper(io);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public boolean atSetpoint() {
        return inputs.currentAngle.isNear(inputs.angleSetpoint, GripperConstants.THRESHOLD);
    }

    public Command setRollerPower(double power) {
        return runOnce(() -> io.setRollerMotorPower(power)).withName("set roller power");
    }

    public Command intake() {
        return setWristPosition(
                        Units.Radians.of(GripperConstants.INTAKE_ANGLE.getRadians()).mutableCopy())
                .alongWith(setRollerPower(GripperConstants.INTAKE_POWER))
                .withName("intake");
    }

    public Command outtake() {
        return setWristPosition(
                        Units.Radians.of(GripperConstants.OUTTAKE_ANGLE.getRadians()).mutableCopy())
                .alongWith(setRollerPower(GripperConstants.OUTTAKE_POWER))
                .withName("outtake");
    }

    public Command setWristPosition(MutableMeasure<Angle> angle) {
        return runOnce(() -> io.setAngle(angle)).withName("set wrist position");
    }

    @Override
    public void periodic() {
        gripperHeight =
                elevatorInputs.currentHeight.mut_plus(
                        Units.Meters.of(GripperConstants.GRIPPER_POSITION_z).mutableCopy());
        gripperPose =
                new Pose3d(
                        GripperConstants.GRIPPER_POSITION0_X,
                        GripperConstants.GRIPPER_POSITION_Y,
                        gripperHeight.in(Units.Meters),
                        new Rotation3d(0, inputs.currentAngle.in(Units.Radians), 0));

        io.updateInputs();
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        gripperLigament.setAngle(new Rotation2d(inputs.currentAngle));
    }
}
