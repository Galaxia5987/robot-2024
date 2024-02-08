package frc.robot;

import edu.wpi.first.units.Units;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;

public class GripperState {
    private Elevator elevator;
    private Gripper gripper;

    public GripperState() {
        elevator = Elevator.getInstance();
        gripper = Gripper.getInstance();
    }

    public boolean isGripperInsideRobot() {
        return gripper.getGripperPosition().getY() + elevator.getCurrentHeight().in(Units.Meters)
                < GripperConstants.GRIPPER_OUTTAKE_MIN_HEIGHT.in(Units.Meters);
    }
}
