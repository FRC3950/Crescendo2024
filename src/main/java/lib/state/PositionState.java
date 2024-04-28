package lib.state;

import lib.pid.TalonConfig;

import java.util.function.DoubleSupplier;

public class PositionState {

    public final TalonConfig talonConfig;

    private DoubleSupplier position;

    public final double epsilon;

    public PositionState(TalonConfig talonConfig, DoubleSupplier position, double epsilon) {
        this.talonConfig = talonConfig;

        this.position = position;

        this.epsilon = epsilon;
    }

    public DoubleSupplier getTargetPosition() {
        return position;
    }

    public PositionState withChangedPosition(DoubleSupplier newPosition) {
        return new PositionState(talonConfig, newPosition, epsilon);
    }

    public void updateGoal(DoubleSupplier position) {
        this.position = position;
    }

    public boolean isReached() {
        return Math.abs(talonConfig.motor.getPosition().getValueAsDouble() - position.getAsDouble()) <= Math.abs(epsilon);
    }
}
