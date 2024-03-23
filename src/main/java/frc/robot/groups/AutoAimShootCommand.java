package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            ).andThen(Commands.waitSeconds(0.25)),

            shooter.shootCommand(intake).withTimeout(1.25),
            pivot.stowCommand()
        );
        addRequirements(pivot, intake, shooter);
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