// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.State;

public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  private final Shooter shooter = Shooter.getInstance();

  public ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setState(State.ACTIVE);
    if (shooter.getRPMS()>65){

      Intake.getInstance().runIndexer();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setState(State.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}