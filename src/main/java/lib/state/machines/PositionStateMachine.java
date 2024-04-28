package lib.state.machines;

/*
*  For use in 2025.
*  */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.state.PositionState;


public abstract class PositionStateMachine extends SubsystemBase implements IStateMachine<PositionState> {

    protected final PositionState initialState;

    protected PositionState currentState;
    protected PositionState goalState;

    protected PositionStateMachine(PositionState initialState){
        this.initialState = initialState;
    }

    public void init() {
        acquireGoalState(initialState);
    }

    public PositionState getGoalState() {
        return goalState;
    }

    protected void applyPosition(PositionState goalState){
        var motor = currentState.talonConfig.motor;
        motor.setControl(goalState.talonConfig.mmVoltage.withPosition(goalState.getTargetPosition().getAsDouble()));
    }

    @Override
    public PositionState getState() {
        return currentState;
    }

    @Override
    public void acquireGoalState(PositionState goalState){
        this.goalState = goalState;

        applyPosition(goalState);
    }

    @Override
    public boolean isAtGoalState(){
        return isAtState(goalState);
    }

    @Override
    public boolean isAtState(PositionState other){
        if(other.isReached()){
            currentState = other;
            return true;
        }

        return false;
    }
}
