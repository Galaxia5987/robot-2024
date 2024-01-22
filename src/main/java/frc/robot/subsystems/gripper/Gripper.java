package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE;
    private final GripperInputs inputs = new GripperInputs();
    private final GripperIO io;
    private final Mechanism2d mechanism = new Mechanism2d(
            3, 3
    );
    private final MechanismRoot2d root = mechanism.getRoot("Gripper", 0, 0);
    private final MechanismLigament2d shoulder = root.append(
            new MechanismLigament2d("Gripper", 0, 0)
    );

    public Gripper(GripperIO io) {
        this.io = io;
    }
}
