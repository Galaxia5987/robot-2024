package frc.robot;

public class RobotState {
    public enum state {
        SHOOT,
        AMP,
        TRAP,
    }

    public static state currentState = state.SHOOT;
}
