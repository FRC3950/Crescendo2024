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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoAimPathCommand extends SequentialCommandGroup {

    DoubleSupplier zero = () -> 0.0;
    BooleanSupplier canShoot = () -> false;

    public AutoAimPathCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drivetrain) {
canShoot = () ->Math.abs(drivetrain.getState().Pose.getY() - 5.55) < 0.5;
        addCommands(
            Commands.either(
                //1st Condition - note is indexed so hold angle and apply velocities
                Commands.parallel(
                    intake.stopCommand(),
                        pivot.setAngleCommand(
                                () -> NoteKinematics.getTargetPivot(() -> NoteKinematics.getAllianceSpeakerDistance(drivetrain)))
                        .andThen(Commands.print("Pivot Angle Set: ")),
                        shooter.applyVelocitiesCommand()
                                ).withTimeout(1.25) //get rid when not in sim
                                .andThen((intake.indexCommand().alongWith(Commands.print("we got a shooter"))).onlyIf(canShoot).withTimeout(0.4))
                                .andThen(Commands.print("End of 1st Condition ")),

                //2nd Condition - note is not indexed so stow and idle
                Commands.parallel(
                    pivot.autoStowCommand(), //remove timeout after sim
                    shooter.idleCommand()
                )               
                    .andThen(intake.intakeCommand()).withTimeout(3)  //need a intteript
                    .andThen(Commands.print("stow/idle/intake on")),
            intake::noteIsIndexed).repeatedly()
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