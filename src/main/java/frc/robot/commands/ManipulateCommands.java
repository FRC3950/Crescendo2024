package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class ManipulateCommands {
    private final Intake intake;
    private final Pivot pivot;
    private final Flipper flipper;

    public ManipulateCommands(Intake intake, Pivot pivot, Flipper flipper) {
        this.intake = intake;
        this.pivot = pivot;
        this.flipper = flipper;
    }

    public Command intakeCommand() {
        return Commands.parallel(
                intake.intakeCommand(),
                pivot.stowCommand()
        );
    }

    public Command stowCommand() {
        return Commands.parallel(
                pivot.stowCommand(),
                flipper.stowCommand()
        );
    }
}
