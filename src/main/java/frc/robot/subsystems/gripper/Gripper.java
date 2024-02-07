package frc.robot.subsystems.gripper;

import edu.wpi.first.math.filter.Debouncer;
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

    @AutoLogOutput private final Mechanism2d mechanism2d = new Mechanism2d(1, 1);
    @AutoLogOutput private Pose3d gripperPose = new Pose3d();

    private final MechanismRoot2d root = mechanism2d.getRoot("Gripper", 0.5, 0.5);
    private final MechanismLigament2d gripperLigament =
            root.append(new MechanismLigament2d("Gripper", 0.3, 0));

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
        return inputs.currentAngle.isNear(
                inputs.angleSetpoint, GripperConstants.THRESHOLD.in(Units.Percent));
    }

    public Command setRollerPower(double power) {
        return runOnce(() -> io.setRollerMotorPower(power)).withName("set roller power");
    }

    public Command intake() {
        return setWristPosition(GripperConstants.INTAKE_ANGLE.mutableCopy())
                .alongWith(setRollerPower(GripperConstants.INTAKE_POWER))
                .withName("intake");
    }

    public Command outtake() {
        return setWristPosition(GripperConstants.OUTTAKE_ANGLE.mutableCopy())
                .alongWith(setRollerPower(GripperConstants.OUTTAKE_POWER))
                .withName("outtake");
    }

    public Command setWristPosition(MutableMeasure<Angle> angle) {
        return runOnce(() -> io.setAngle(angle)).withName("set wrist position");
    }

    @Override
    public void periodic() {
        MutableMeasure<Distance> gripperHeight =
                elevatorInputs.gripperHeight.mut_plus(GripperConstants.GRIPPER_POSITION_z);
        gripperPose =
                new Pose3d(
                        GripperConstants.GRIPPER_POSITION_X.in(Units.Meters),
                        GripperConstants.GRIPPER_POSITION_Y.in(Units.Meters),
                        gripperHeight.in(Units.Meters),
                        new Rotation3d(0, inputs.currentAngle.in(Units.Radians), 0));

        io.updateInputs();
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        gripperLigament.setAngle(new Rotation2d(inputs.currentAngle));
    }
}
