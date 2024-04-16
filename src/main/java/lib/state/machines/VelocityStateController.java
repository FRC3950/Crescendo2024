package lib.state.machines;

import lib.state.VelocityState;



public abstract class VelocityStateController extends Stateful<VelocityState>  {

    protected final VelocityState initialState;

    protected VelocityState currentState;
    protected VelocityState goalState;

    protected VelocityStateController(VelocityState initialState){
        this.initialState = initialState;
        currentState = initialState;
    }

    protected void applyVelocities(VelocityState goalState){
        goalState.velocities.forEach((config, vel) -> {
            config.motor.setControl(config.velVoltage.withVelocity(vel.getAsDouble()));
        });
    }

    @Override
    public void setState(VelocityState goalState) {
        this.goalState = goalState;
        applyVelocities(goalState);
    }

    @Override
    public boolean isAtState(VelocityState state) {

        var config = state.getTalonSetup();

        if(config.motor.getVelocity().getValueAsDouble() >= state.velocities.get(config).getAsDouble()){
            currentState = goalState;
            return true;
        }

        return false;
    }

    @Override
    public VelocityState getState() {
        return currentState;
    }
}
