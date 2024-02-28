// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class alignWithStage_Odometry extends Command {
  private Pose2d currentPose;
  private PathPlannerPath upperStage;
  private PathPlannerPath lowerStage;
  /** Creates a new alignWithStage_Odometry. */
  public alignWithStage_Odometry(Pose2d currentPose) {

    PathPlannerPath upperStage = PathPlannerPath.fromPathFile("UpperStage");
    PathPlannerPath lowerStage = PathPlannerPath.fromPathFile("LowerStage");

    this.currentPose = currentPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(currentPose.getY() > 4.0){

      AutoBuilder.followPath(upperStage);
    }
    else{
      AutoBuilder.followPath(lowerStage);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
