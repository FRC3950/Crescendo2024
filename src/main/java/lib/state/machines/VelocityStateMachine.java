package lib.state.machines;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.pid.TalonConfig;
import lib.state.VelocityState;

import java.util.HashSet;
import java.util.function.DoubleSupplier;


/**
 * A virtual state machine for controlling the velocity of motors. <br> <br>
 * Implements {@link IStateMachine} with generic type {@link HashSet<VelocityState>}.
 * The {@link HashSet} generic type allows for the simultaneous control of multiple motors within a subsystem.
 * @see lib.state.machines.IStateMachine
 * @see VelocityState
 * */
public abstract class VelocityStateMachine extends SubsystemBase implements IStateMachine<HashSet<VelocityState>> {

    protected final HashSet<VelocityState> initialState;

    protected HashSet<VelocityState> currentState;
    protected HashSet<VelocityState> goalState;

    /**
     * Initializes the subsystem with an initial state.
     * @param initialState a hash set of the initial {@link VelocityState} objects the subsystem will acquire upon robot enable.
     * */
    protected VelocityStateMachine(HashSet<VelocityState> initialState){
        this.initialState = initialState;
    }

    private void applyVelocities(HashSet<VelocityState> goalState, boolean isNegative){
        goalState.forEach(state -> {
            final TalonConfig config = state.talonConfig;
            final double velocity = state.getTargetVelocity().getAsDouble() * ((isNegative) ?  -1 : 1);

            config.motor.setControl(config.velVoltage.withVelocity(velocity));
        });
    }

    protected void updateGoalState(DoubleSupplier newValue) {
        goalState.forEach(s -> s.updateGoal(newValue));
    }

    /**
     * Functions as {@link #acquireGoalState}, but the target velocities of all applied states are negative.
     * @param goalState a hash set of the {@link VelocityState} objects that will be applied to each {@link VelocityState#talonConfig}.
     * @see #acquireGoalState
     */
    protected void acquireNegativeGoalState(HashSet<VelocityState> goalState){
        this.goalState = goalState;
        applyVelocities(goalState, true);
    }

    /**
     * Applies the target velocities of a {@link HashSet<VelocityState>} of {@link VelocityState} objects to each {@link VelocityState#talonConfig}.
     * @param goalState a hash set of the {@link VelocityState} objects that will be applied to each {@link VelocityState#talonConfig}.
     */
    @Override
    public void acquireGoalState(HashSet<VelocityState> goalState){
        this.goalState = goalState;
        applyVelocities(goalState, false);
    }

    @Override
    public HashSet<VelocityState> getState() {
        return currentState;
    }

    /**
     * Checks if the subsystem has reached its goal state. If it has, {@link #currentState} is assigned to {@link #goalState} and true is returned.
     * @return Returns whether the subsystem has reached the goal state as defined by {@link VelocityState#epsilon}
     * */
    @Override
    public boolean isAtGoalState() {
        if(isAtState(goalState)){
            currentState = goalState;
            return true;
        }

        return false;
    }

    @Override
    public boolean isAtState(HashSet<VelocityState> state) {
        for(VelocityState vState : state){
            if(!vState.isReached())
                return false;
        }

        currentState = state;
        return true;
    }
}
