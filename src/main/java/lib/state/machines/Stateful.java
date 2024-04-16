package lib.state.machines;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Stateful<S> extends SubsystemBase {
    public abstract S getState();

    public abstract void setState(S goalState);

    public abstract boolean isAtState(S state);

    public void removeState(S toRemove){}

    public void addState(S toAdd){};
}
