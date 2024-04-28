package lib.state.examples;

import lib.pid.MotionMagicConfig;
import lib.state.PositionState;
import lib.state.VelocityState;
import lib.pid.PidConfig;
import lib.pid.TalonConfig;

import java.util.HashSet;
import java.util.function.DoubleSupplier;

public interface DummyConstants {
    final class VelocityExample {
        static final double kP = 1;
        static final double kV = 1;

        static final TalonConfig shooterConfig = new TalonConfig(40, "CANivore", new PidConfig(kP, kV, 0, 0, 0, 0, false));
        static final TalonConfig indexerConfig = new TalonConfig(39, "CANivore", new PidConfig(0, 0, 0, 0, 0, 0, false));

        static final HashSet<VelocityState> shootRamp = new HashSet<>(){{
            add(new VelocityState(shooterConfig, () -> 10, 1));
        }};

        static final HashSet<VelocityState> shoot = new HashSet<>(){{
           add(new VelocityState(shooterConfig, () -> 10, 1));
           add(new VelocityState(indexerConfig, () -> 15, 1));
        }};

        static final HashSet<VelocityState> idle = new HashSet<>(){{
            add(new VelocityState(shooterConfig, () -> 0, 1));
            add(new VelocityState(indexerConfig, () -> 0, 1));
        }};
    }

    final class PositionExample {

        public static final double kA = 1; // FIXME
        public static final double jerk = 10;
        public static final double accel = 10;
        public static final double vel = 10;

        public static final PidConfig config = new PidConfig(
                0, 0, 0, 0, 0, 0, false
        );

        public static final MotionMagicConfig mmConfig = new MotionMagicConfig(
                kA, accel, vel, jerk
        );

        public static final DoubleSupplier stowPosition = () -> 0.5;
        public static final DoubleSupplier ampPosition = () -> -1.25;

        public static final PositionState stowed = new PositionState(
                new TalonConfig(0, "CANivore", config, mmConfig),
                stowPosition,
                0.05
        );

        public static final PositionState amped = stowed.withChangedPosition(ampPosition);

    }
}
