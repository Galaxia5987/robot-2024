package frc.robot.scoreStates;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ScoreStateContext {
    private static ScoreState currentState;
    private static ScoreStateContext INSTANCE;

    public static ScoreStateContext getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ScoreStateContext();
        }
        return INSTANCE;
    }

    public Command setCurrentState(ScoreState state) {
        return Commands.sequence(
                Commands.runOnce(() -> currentState = state),
                currentState.calculateTargets(),
                currentState.prepareSubsystems());
    }

    public Command updateSubsystems() {
        return currentState.calculateTargets().andThen(currentState.prepareSubsystems());
    }

    public Command score() {
        return currentState.score();
    }
}
