package frc.robot.scoreStates;

public class ScoreStateContext {
    private static ScoreState currentState;
    private static ScoreStateContext INSTANCE;

    public ScoreStateContext getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ScoreStateContext();
        }
        return INSTANCE;
    }
}
