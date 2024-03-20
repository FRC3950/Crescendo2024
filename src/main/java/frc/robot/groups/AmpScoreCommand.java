// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScoreCommand extends SequentialCommandGroup {
  /** Creates a new AmpScoreCommand. */
  public AmpScoreCommand(Pivot pivot, Flipper flipper) {
    addCommands(
      pivot.setAngleCommand(() -> 50),
      flipper.ampCommand()
    ); // Stow command runs on button false (See RobotContainer)

    addRequirements(pivot, flipper);
  }
}
