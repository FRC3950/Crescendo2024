package frc.robot.groups.shoot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import lib.odometry.ScoringKinematics;

import java.util.function.DoubleSupplier;

public class AutoAimShootCommand extends SequentialCommandGroup {
    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drive, DoubleSupplier angle) {
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(angle),
                        shooter.applyShootState() // Finishes when at velocity
                ).withTimeout(1.25).andThen(Commands.waitSeconds(0.25)),

                // shooter.shootCommand(intake, Constants.Shooter.activeSpeed, drive).withTimeout(1.25),
                pivot.stowInstantCommand()
        );
        addRequirements(pivot, intake, shooter);
    }

    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, Swerve drivetrain) {
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(
                                () -> ScoringKinematics.getTargetPivot(() -> ScoringKinematics.getAllianceSpeakerDistance(drivetrain))
                        ),

                        shooter.applyShootState()
                ).withTimeout(2) //added for sim
                //Commands.waitSeconds(0.25),
//                shooter.shootCommand(intake, Constants.Shooter.activeSpeed, drivetrain).until(()->!intake.noteIsIndexed()).withTimeout(1)
//                .andThen(Commands.waitSeconds(0.25)),
//                pivot.stowInstantCommand()
        );
    }
}