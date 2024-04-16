package frc.robot.groups.shoot;

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
                ).withTimeout(1.25).andThen(Commands.waitSeconds(0.25)),

                shooter.shootCommand(intake, Constants.Shooter.activeSpeed, drive).withTimeout(1.25),
                pivot.stowCommand()
        );
        addRequirements(pivot, intake, shooter);
    }

    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drivetrain) {
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(
                                () -> NoteKinematics.getTargetPivot(() -> NoteKinematics.getAllianceSpeakerDistance(drivetrain))
                        ),

                        shooter.applyVelocitiesCommand()
                ).withTimeout(2), //added for sim
                //Commands.waitSeconds(0.25),
                shooter.shootCommand(intake, Constants.Shooter.activeSpeed, drivetrain).until(()->!intake.noteIsIndexed()).withTimeout(1)
                .andThen(Commands.waitSeconds(0.25)),
                pivot.stowCommand()
        );
    }
}