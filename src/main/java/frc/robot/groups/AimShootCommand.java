package frc.robot.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class AimShootCommand extends SequentialCommandGroup {
    public AimShootCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
        addCommands(
                pivot.commands.setAngleCommand(angle),
                shooter.commands.shoot(intake).withTimeout(1.25),
                intake.commands.stopCommand(),
                pivot.commands.stowCommand()
        );
    }
}