// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.State;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */

  private final Intake intake = Intake.getInstance();
  private boolean isForward = false;

  public IntakeCommand(boolean isForward) {
    this.isForward = isForward; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

            intake.setState((isForward) ? State.INTAKE : State.OUTTAKE, isForward);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake.noteIsIndexed())
    {return true;
    
    }
    return false;
  }
}
