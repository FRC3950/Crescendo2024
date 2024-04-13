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

public class AutoAimDuringPathFollowing extends SequentialCommandGroup {
  DoubleSupplier zero = () -> -0.0;

    public AutoAimDuringPathFollowing(Pivot pivot, Intake intake, Shooter shooter, Swerve drivetrain,
                               Supplier<DriverStation.Alliance> alliance, Supplier<Translation2d> redSpeaker, Supplier<Translation2d> blueSpeaker) {
        var activeSpeaker = alliance.get() == DriverStation.Alliance.Red ? redSpeaker.get() : blueSpeaker.get();
        addCommands(
          Commands.either(
            
          
                Commands.parallel(
                        pivot.setAngleCommand(
                                () -> NoteKinematics.getTargetPivot(() -> drivetrain.getState()
                                .Pose.getTranslation()
                                .getDistance(activeSpeaker))
                        ).andThen(Commands.print("Pivot Angle Set: ")),
                        shooter.applyVelocitiesCommand().andThen(Commands.print("Shooter Velocities Set: "))
                                  ).repeatedly().andThen(Commands.print("Command Repeats: "))
          
          , 
          
          Commands.parallel(
            pivot.stowCommand(),
            shooter.idleCommand()
          ).until(()->pivot.isAtAngle(zero)).andThen(Commands.waitSeconds(0.25))
          .andThen( intake.intakeCommand()).andThen(Commands.print("stow/idle"))
          
          
          , ()-> intake.noteIsIndexed())
                                

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