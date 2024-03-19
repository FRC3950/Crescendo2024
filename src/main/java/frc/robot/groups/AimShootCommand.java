package frc.robot.groups;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class AimShootCommand extends SequentialCommandGroup {
    public AimShootCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
        addCommands(
            Commands.parallel(
                pivot.setAngleCommand(angle),
                shooter.applyVelocitiesCommand() // Finishes when at velocity   
            ),
            shooter.shootCommand(intake)
        );
        addRequirements(pivot, intake, shooter);
    }
}