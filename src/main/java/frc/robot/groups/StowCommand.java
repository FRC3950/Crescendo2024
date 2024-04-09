// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class StowCommand extends SequentialCommandGroup {
    /**
     * Creates a new StowCommand.
     */
    public StowCommand(Pivot pivot, Intake intake, Shooter shooter, Flipper flipper) {
        addCommands(
                flipper.stowCommand(),
                pivot.stowCommand(),
                intake.stopCommand(),
                shooter.idleCommand()
        );

        addRequirements(pivot, intake, shooter, flipper);
    }
}
