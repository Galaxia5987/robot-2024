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

    public void setCurrentState(ScoreState state) {
        currentState = state;
        currentState.calculateTargets();
        currentState.prepareSubsystems();
    }

    public void updateSubsystems() {
        currentState.calculateTargets();
        currentState.prepareSubsystems();
    }

    public void score() {
        currentState.score();
    }
}
