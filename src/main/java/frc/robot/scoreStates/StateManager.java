package frc.robot.scoreStates;

public class StateManager {
    private static StateManager INSTANCE;
    private static ScoreState currentState;

    private StateManager(){

    }

    private StateManager(ScoreState initializedState) {
        currentState = initializedState;
    }

    public static StateManager getINSTANCE(ScoreState initializedState) {
        if (INSTANCE == null) {
            INSTANCE = new StateManager(initializedState);
        }
        return INSTANCE;
    }

    public static StateManager getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new StateManager();
        }
        return INSTANCE;
    }

    public static boolean isState(ScoreState isState) {
        return currentState == isState;
    }

    public static void setCurrentState(ScoreState currentState) {
        StateManager.currentState = currentState;
    }

    public static ScoreState getCurrentState() {
        return currentState;
    }
}
