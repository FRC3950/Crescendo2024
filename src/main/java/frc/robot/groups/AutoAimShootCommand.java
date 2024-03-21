package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class AutoAimShootCommand extends SequentialCommandGroup {
    public AutoAimShootCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
        addCommands(
                Commands.parallel(
                        pivot.setAngleCommand(angle),
                        shooter.applyVelocitiesCommand() // Finishes when at velocity
                ),

                shooter.shootForAutoCommand(intake).withTimeout(2),
                shooter.idleCommand(),
                intake.stopCommand(),
                pivot.stowCommand());

        addRequirements(pivot, intake, shooter);
    }
}