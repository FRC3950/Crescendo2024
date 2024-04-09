package frc.robot.groups;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

public class AimShootCommand extends SequentialCommandGroup {
    public AimShootCommand(Pivot pivot, Intake intake, Swerve drive, Shooter shooter, DoubleSupplier angle) {
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(angle),
                        shooter.applyVelocitiesCommand() // Finishes when at velocity
                ).andThen(Commands.waitSeconds(0.25)),
                shooter.shootCommand(intake, Constants.Shooter.activeSpeed, drive)
        );
        addRequirements(pivot, intake, shooter);
    }
}