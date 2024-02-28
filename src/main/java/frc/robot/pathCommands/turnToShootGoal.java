// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class turnToShootGoal extends Command {
  /** Creates a new turnToShootGoal. */

  private final Swerve m_swerve;
       Pose2d redSpeakerPose = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(180));
     Pose2d blueSpeakerPose = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));

  public turnToShootGoal(Swerve swerve) {
    m_swerve = swerve;
   

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


var angleFromShot =      
Math.atan(
    (m_swerve.getState().Pose.relativeTo(blueSpeakerPose).getY()
    /m_swerve.getState().Pose.relativeTo(blueSpeakerPose).getX()) 
    
)* 180 / Math.PI;

System.out.println("Angle from shot: " + angleFromShot);

     var pathPoints =PathPlannerPath.bezierFromPoses(
      m_swerve.getState().Pose,
      m_swerve.getState().Pose.transformBy(new Transform2d(1, 1, Rotation2d.fromDegrees(0)))
    );
    System.out.println("Following Path");
    System.out.println(pathPoints);

    var pathfindThenFollowPath = new PathPlannerPath(
        pathPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(angleFromShot)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    System.out.println("Following Path");
    System.out.println(pathfindThenFollowPath.getGoalEndState());

    AutoBuilder.followPath(pathfindThenFollowPath       );


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
