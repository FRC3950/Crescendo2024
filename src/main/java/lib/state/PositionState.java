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

    @Override // Autogenerated, changed talonConfig comparison
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        PositionState that = (PositionState) o;

        if (Double.compare(epsilon, that.epsilon) != 0) return false;
        if (talonConfig.motor.getDeviceID() != that.talonConfig.motor.getDeviceID()) return false;
        return position.equals(that.position);
    }

    @Override // Autogenerated
    public int hashCode() {
        int result;
        long temp;
        result = talonConfig.hashCode();
        result = 31 * result + position.hashCode();
        temp = Double.doubleToLongBits(epsilon);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
