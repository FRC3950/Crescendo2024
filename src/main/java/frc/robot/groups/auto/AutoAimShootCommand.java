package frc.robot.groups.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.NoteKinematics;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoAimShootCommand extends SequentialCommandGroup {
    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
        addCommands(
            Commands.parallel(
                pivot.setAngleCommand(angle),
                shooter.applyVelocitiesCommand() // Finishes when at velocity   
            ).andThen(Commands.waitSeconds(0.25)),

            shooter.shootCommand(intake).withTimeout(1.25),
            pivot.stowCommand()
        );
        addRequirements(pivot, intake, shooter);
    }

    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drivetrain, 
        Supplier<DriverStation.Alliance> alliance, Supplier<Translation2d> redSpeaker, Supplier<Translation2d> blueSpeaker){
        var activeSpeaker = alliance.get() == DriverStation.Alliance.Red ? redSpeaker.get() : blueSpeaker.get();
        addCommands(
            Commands.parallel(
                pivot.setAngleCommand(
                    () -> NoteKinematics.getTargetPivot(() -> drivetrain.getState().Pose.getTranslation().getDistance(activeSpeaker))
                ),
                shooter.applyVelocitiesCommand()
            ),
            Commands.waitSeconds(0.7),
            shooter.shootCommand(intake).withTimeout(1),
            pivot.stowCommand()
        );
    }
}

// public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
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
//     }