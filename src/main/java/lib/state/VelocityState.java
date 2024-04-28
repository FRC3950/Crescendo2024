package lib.state;

import lib.pid.TalonConfig;
import lib.state.machines.VelocityStateMachine;

import java.util.HashSet;
import java.util.function.DoubleSupplier;

public class VelocityState {

    public final TalonConfig talonConfig;

    private DoubleSupplier targetVelocity;

    public final double epsilon;

    public VelocityState(TalonConfig talonConfig, DoubleSupplier targetVelocity, double epsilon) {
        this.talonConfig = talonConfig;
        this.targetVelocity = targetVelocity;
        this.epsilon = epsilon;
    }

    public DoubleSupplier getTargetVelocity() {
        return targetVelocity;
    }

    public VelocityState withChangedVelocity(DoubleSupplier newVelocity){
        return new VelocityState(talonConfig, newVelocity, epsilon);
    }

    public void updateGoal(DoubleSupplier targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public HashSet<VelocityState> asHashSet(){
         return new HashSet<>() {{
            add(VelocityState.this);
        }};
    }

    public VelocityState stopped() {
        return withChangedVelocity(() -> 0);
    }

    public VelocityState negative() {
        return new VelocityState(talonConfig, () -> -targetVelocity.getAsDouble(), epsilon);
    }

    public boolean isReached() {
        var feedback = talonConfig.motor.getVelocity().getValueAsDouble();

        return Math.abs(feedback - targetVelocity.getAsDouble()) <= Math.abs(epsilon);
    }
}
