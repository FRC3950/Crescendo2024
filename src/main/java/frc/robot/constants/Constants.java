package frc.robot.constants;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public interface Constants {

        final class Pivot {
            public static final int id = 15;

            public static final double kP = 2.5;
            public static final double kV = 0.1185;
            public static final double kG = 0;

            public static final DoubleSupplier stowPosition = () -> -2;
          //  public static final DoubleSupplier shuttlePosition = () -> Shuffleboard.getTab("Tab 5").add("Shuttle Position", 30).getEntry().getDouble(30);
        }

        final class Climber {
            public static final int motor1Id = 19;
            public static final int motor2Id = 49;

            public static final double kP = 0.0005;
            public static final double kV = 0.05;
            public static final double kG = 0.05;
        }

        final class Flipper {
            public static final int id = 31;

            public static final double kP = 12;
            public static final double kV = 0.12;
            public static final double kG = 0.2;

            public static final DoubleSupplier stowPosition = () -> 0.5;
            public static final DoubleSupplier ampPosition = () -> -1.25;
        }

        final class Elevator {
            public static final int leftId = 49;
            public static final int rightId = 19;
        }

        final class Intake {
            public static final int rightId = 42;
            public static final int leftId = 41;
            public static final int indexerId = 14;

            public static final double indexerKp = 0.01;
            public static final double indexerKv = 0.12;

            public static final double intakeKp = 0.764999999888241291;
            //public static final double intakeKs = 7.1;
            public static final double intakeKv = 0.0;

            public static final DoubleSupplier indexerActiveVelocity = () -> 38;
            public static final DoubleSupplier intakeActiveVelocity = () -> 65;
            public static final DoubleSupplier ampActiveVelocity = () -> -40;
        }

        final class Shooter {
            public static final int topId = 17;
            public static final int bottomId = 13;

            public static final double kP = 0.005;
            public static final double kV = 0.112;

            public static final DoubleSupplier activeSpeed = () -> 75;
            public static final DoubleSupplier idleSpeed = () -> 3;
            public static final DoubleSupplier ampSpeed = () -> 18;
         //   public static final DoubleSupplier shuttleSpeed = () -> Shuffleboard.getTab("Tab 5").add("Shuttle Speed", 75).getEntry().getDouble(75);
        }
}
