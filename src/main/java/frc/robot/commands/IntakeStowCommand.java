// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.teleop.IntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeStowCommand extends ParallelCommandGroup {
  /** Creates a new IntakeStowCommand. */

  public IntakeStowCommand() {
    addCommands(
      new IntakeCommand(true)
    );
  }
}
