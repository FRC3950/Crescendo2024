package lib.state.examples;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.log.Loggable;
import lib.state.PositionState;
import lib.state.machines.PositionStateMachine;

public class PositionExample extends PositionStateMachine implements Loggable {

    private final PositionState stowed = DummyConstants.PositionExample.stowed;
    private final PositionState amped = DummyConstants.PositionExample.amped;

    public PositionExample() {
        super(DummyConstants.PositionExample.stowed);
    }

    @Override
    public void log() {}

    public Command ampCommand() {
        return new Command() {

            @Override
            public void initialize() {
                acquireGoalState(amped);
            }

            @Override
            public boolean isFinished() {
                return isAtGoalState();
            }
        };
    }

    public Command stowCommand() {
        return Commands.runOnce(() -> acquireGoalState(stowed));
    }
}
