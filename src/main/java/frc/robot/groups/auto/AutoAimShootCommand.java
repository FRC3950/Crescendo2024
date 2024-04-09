package frc.robot.groups.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import lib.odometry.NoteKinematics;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoAimShootCommand extends SequentialCommandGroup {
    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drive, DoubleSupplier angle) {
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(angle),
                        shooter.applyVelocitiesCommand() // Finishes when at velocity
                ).andThen(Commands.waitSeconds(0.25)),

                shooter.shootCommand(intake, Constants.Shooter.activeSpeed, drive).withTimeout(1.25),
                pivot.stowCommand()
        );
        addRequirements(pivot, intake, shooter);
    }

    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drivetrain,
                               Supplier<DriverStation.Alliance> alliance, Supplier<Translation2d> redSpeaker, Supplier<Translation2d> blueSpeaker) {
        var activeSpeaker = alliance.get() == DriverStation.Alliance.Red ? redSpeaker.get() : blueSpeaker.get();
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(
                                () -> NoteKinematics.getTargetPivot(() -> drivetrain.getState().Pose.getTranslation().getDistance(activeSpeaker))
                        ),
                        shooter.applyVelocitiesCommand()
                ),
                Commands.waitSeconds(0.7),
                shooter.shootCommand(intake, Constants.Shooter.activeSpeed, drivetrain).withTimeout(1),
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