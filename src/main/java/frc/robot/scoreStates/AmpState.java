package frc.robot.scoreStates;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commandGroups.CommandGroups;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class AmpState implements ScoreState {
    private CommandGroups commandGroups;
    private Conveyor conveyor;
    private Shooter shooter;
    private Gripper gripper;
    private Hood hood;

    public AmpState() {
        commandGroups = CommandGroups.getInstance();
        shooter = Shooter.getInstance();
        gripper = Gripper.getInstance();
        hood = Hood.getInstance();
        conveyor = Conveyor.getInstance();
    }

    @Override
    public Command score() {
        return Commands.sequence(
                        shooter.setVelocity(
                                        ShooterConstants.TOP_AMP_VELOCITY,
                                        ShooterConstants.BOTTOM_AMP_VELOCITY)
                                .alongWith(
                                        hood.setAngle(HoodConstants.AMP_ANGLE)
                                                .until(shooter::atSetpoint),
                                        gripper.setRollerPower(GripperConstants.INTAKE_POWER)
                                                .withTimeout(1),
                                        gripper.stopGripper()))
                .alongWith(conveyor.setVelocity(ConveyorConstants.AMP_VELOCITY));
    }
}
