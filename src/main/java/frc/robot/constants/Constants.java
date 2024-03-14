package frc.robot.constants;

import java.util.function.DoubleSupplier;

public interface Constants {

        public static final class Pivot {
            public static final int id = 15;
        }

        public static final class Flipper {
            public static final int id = 31;

            public static final DoubleSupplier stowPosition = () -> 0.4;
            public static final DoubleSupplier ampPosition = () -> -1.15;
        }

        public static final class Elevator {
            public static final int leftId = 49;
            public static final int rightId = 19;
        }

        public static final class Intake {
            public static final int rightId = 42;
            public static final int leftId = 41;
            public static final int indexerId = 14;

            public static final double indexerKp = 0.01;
            public static final double indexerKv = 0.12;

            public static final double intakeKp = 0.02;
            public static final double intakeKv = 0.12;

            public static final DoubleSupplier indexerActiveVelocity = () -> 38;
            public static final DoubleSupplier intakeActiveVelocity = () -> 38; 
        }
    
        public static final class Shooter {
            public static final int topId = 17;
            public static final int bottomId = 13;

            public static final double kP = 0.005;
            public static final double kV = 0.12;

            public static final DoubleSupplier activeSpeed = () -> 80;
            public static final DoubleSupplier idleSpeed = () -> 3;
            public static final DoubleSupplier ampSpeed = () -> 18;
        }
}
