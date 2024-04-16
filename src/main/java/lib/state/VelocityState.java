package lib.state;

import lib.system.pid.TalonConfig;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public class VelocityState {

    public Map<TalonConfig, DoubleSupplier> velocities = new HashMap<>();

    private TalonConfig talonSetup;

    public VelocityState(TalonConfig talonSetup, DoubleSupplier targetVelocity){
        this.talonSetup = talonSetup;

        velocities.put(talonSetup, targetVelocity);
    }

    public VelocityState(Map<TalonConfig, DoubleSupplier> velocities){
        this.velocities = velocities;
    }

    public VelocityState withChangedVelocity(DoubleSupplier newVelocity){
        return new VelocityState(talonSetup, newVelocity);
    }

    public TalonConfig getTalonSetup() {
        return talonSetup;
    }
}
