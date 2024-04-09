package frc.robot.groups.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;


public class AutoAimCommand extends SequentialCommandGroup {
    public AutoAimCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(angle),
                        shooter.applyVelocitiesCommand() // Finishes when at velocity
                )
        );
        addRequirements(pivot, intake, shooter);
    }
}
