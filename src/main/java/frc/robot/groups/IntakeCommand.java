// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class IntakeCommand extends ParallelCommandGroup {
    /**
     * Creates a new IntakeStowCommand.
     */
    public IntakeCommand(Pivot pivot, Intake intake) {
        addCommands(
                intake.intakeCommand(),
                pivot.stowCommand()
        );
    }
}
