package frc.robot.groups.shoot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import lib.odometry.NoteKinematics;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoAimPathCommand extends SequentialCommandGroup {

    DoubleSupplier zero = () -> 0.0;

    public AutoAimPathCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drivetrain) {

        addCommands(
            Commands.either(
                Commands.parallel(
                        pivot.setAngleCommand(
                                () -> NoteKinematics.getTargetPivot(() -> NoteKinematics.getAllianceSpeakerDistance(drivetrain))
                        ).andThen(Commands.print("Pivot Angle Set: ")),
                        shooter.applyVelocitiesCommand()
                                .andThen(Commands.print("Shooter Velocities Set: "))).repeatedly()
                                .andThen(Commands.print("Command Repeats: ")),
                Commands.parallel(
                    pivot.autoStowCommand(),
                    shooter.idleCommand()
                )
                    .withTimeout(0.75)
                    .andThen(intake.intakeCommand())
                    .andThen(Commands.print("stow/idle")),
            intake::noteIsIndexed)
        );
    }
}

/* public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
//         addCommands(
//                 Commands.parallel(
//                         pivot.setAngleCommand(angle),
//                         shooter.applyVelocitiesCommand() // Finishes when at velocity
//                 ),

//                 shooter.shootForAutoCommand(intake).withTimeout(1.25),
//                 shooter.idleCommand(),
//                 intake.stopCommand(),
//                 pivot.stowCommand());

//         addRequirements(pivot, intake, shooter);
//     } */