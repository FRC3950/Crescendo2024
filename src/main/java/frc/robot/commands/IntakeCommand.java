// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  /** Creates a new Intake. */
  
  private final Intake s_Intake = Intake.getInstance();

  public IntakeCommand() {
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.intake(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
