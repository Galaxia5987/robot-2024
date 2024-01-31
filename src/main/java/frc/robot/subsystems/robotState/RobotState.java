package frc.robot.subsystems.robotState;

public class RobotState {
    public enum state {
        SHOOT,
        AMP,
        TRAP,
    }

    public static state currentState = state.SHOOT;
}
