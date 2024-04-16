package lib.state.examples;

import lib.state.VelocityState;
import lib.system.pid.PidConfig;
import lib.system.pid.TalonConfig;

import java.util.HashMap;

public interface DummyConstants {
    final class VelocityExample {
        static final double kP = 1;
        static final double kV = 1;

        static final TalonConfig shooterConfig = new TalonConfig(40, "CANivore", new PidConfig(kP, kV, 0, 0, 0, 0, false));
        static final TalonConfig indexerConfig = new TalonConfig(39, "CANivore", new PidConfig(0, 0, 0, 0, 0, 0, false));

        static final VelocityState shootRamp = new VelocityState(shooterConfig, () -> 40);

        static final VelocityState shoot = new VelocityState(new HashMap<>() {{
            put(shooterConfig, () -> 40);
            put(indexerConfig, () -> 10);
        }});

        static final VelocityState idle = new VelocityState(new HashMap<>() {{
            put(shooterConfig, () -> 5);
            put(indexerConfig, () -> 0);
        }});

        static final VelocityState lob = new VelocityState(new HashMap<>() {{
            put(shooterConfig, () -> 10);
            put(indexerConfig, () -> 10);
        }});
    }
}
