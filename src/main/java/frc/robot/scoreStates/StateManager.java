package frc.robot.scoreStates;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;

public class StateManager {
    private static StateManager INSTANCE;
    @Getter private ScoreState currentState;

    private StateManager() {}

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

    public boolean isState(ScoreState isState) {
        return currentState == isState;
    }

    public Command setCurrentState(ScoreState state) {
        return Commands.runOnce(() -> currentState = state);
    }
}
