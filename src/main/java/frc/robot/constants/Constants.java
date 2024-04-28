package frc.robot.constants;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import lib.pid.MotionMagicConfig;
import lib.pid.PidConfig;
import lib.pid.TalonConfig;
import lib.state.PositionState;

import java.util.function.DoubleSupplier;

public interface Constants {
    final class Physics {
        public static final double noteExitVelocity = 10; // Immediately after leaving shooter

        public static final double targetSpeakerHeight = 1.75;

        public static final double lobVertexHeight = 3;
        public static final double yVelocity = Math.sqrt(9.81 * 2 * lobVertexHeight);
        public static final double airTime = 1.564;
    }

    final class Pivot {
        public static final int id = 15;
        public static final int cancoderId = 52;

        public static final double kD = 0.00;
        public static final double kV = 24.75;
        public static final double kP = 100;

        public static final double kG = 0.3;

        public static final DoubleSupplier stowPosition = () -> -0.02;
        public static final DoubleSupplier ampPosition = () -> 0.3;
        public static final DoubleSupplier speakerPosition = () -> 0.1;

        public static final MotionMagicConfig mmConfig = new MotionMagicConfig(0, 0, 0, 0); //FIXME

        public static final TalonConfig config = new TalonConfig(
          id, "CANivore", new PidConfig(kP, 0, kD, kV, 0, kG, false), mmConfig
        );
    }

    final class Climber {
        public static final int motor1Id = 19;
        public static final int motor2Id = 49;
    }

    final class Flipper {
        public static final int id = 31;

        public static final double kP = 12;
        public static final double kV = 0.12;
        public static final double kG = 0.2;

        public static final MotionMagicConfig mmConfig = new MotionMagicConfig(0, 0, 0, 0); //FIXME

        public static final TalonConfig config = new TalonConfig(
            id, "CANivore", new PidConfig(kP, 0, 0, kV, 0, kG, false), mmConfig
        );

    }

    final class Intake {
        public static final int rightId = 42;
        public static final int leftId = 41;
        public static final int indexerId = 14;

        public static final double indexerKp = 0.01;
        public static final double indexerKv = 0.12;

        public static final double intakeKp = 0.1;
        public static final double intakeKv = 0.115;

        public static final DoubleSupplier indexerActiveVelocity = () -> 38;
        public static final DoubleSupplier intakeActiveVelocity = () -> 60;
        public static final DoubleSupplier ampActiveVelocity = () -> -40;

        public static final TalonConfig intakeConfig = new TalonConfig(
                42, "rio", new PidConfig(intakeKp, 0, 0, intakeKv, 0, 0, false)
        ).withFollower(41, "rio", false);

        public static final TalonConfig indexerConfig = new TalonConfig(
                14, "CANivore", new PidConfig(indexerKp, 0, 0, indexerKv, 0, 0, false)
        );
    }

    final class Shooter {
        public static final int topId = 17;
        public static final int bottomId = 13;

        public static final double kP = 0.005;
        public static final double kV = 0.112;

        public static final DoubleSupplier activeSpeed = () -> 75;
        public static final DoubleSupplier idleSpeed = () -> 5;

        public static final TalonConfig config = new TalonConfig(
                topId, "CANivore", new PidConfig(kP, 0, 0, kV, 0, 0, false)
        ).withFollower(bottomId, "CANivore", true);
    }
}
