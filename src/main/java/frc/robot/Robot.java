// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean UseLimelight = true;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);



//     for (int port = 5800; port <= 5805; port++) {
// PortForwarder.add(port, "limelight.local", port);
// }

  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //SmartDashboard.putNumber("AA_Angle off X axis", m_robotContainer.drivetrain.getState().Pose.getTranslation().getAngle().getDegrees()); 


    // if (false) {    
      
    //   var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
    //   LimelightHelpers.setLEDMode_ForceOn("limelight");
    //   LimelightHelpers.setLEDMode_ForceBlink("limelight");

    //   Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
    //  // System.out.println("///////////////////");

    //   if (lastResult.valid) {
    //     m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
    //          // System.out.println("..............");

    //   }

    
    // }

    // if (false) {    
      
    //   var x = Timer.getFPGATimestamp() / 10;
    //   x += 0.1 * Math.random();
    //   var y = Timer.getFPGATimestamp() / 10;
    //   y += 0.1 * Math.random();

    //   Pose2d llPose = new Pose2d(x, y, Rotation2d.fromDegrees(90));

    //   if (true) {
    //     m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp(), new Matrix<>(Nat.N3(), Nat.N1()) );
    //     //System.out.println("..............");

    //     // may have to do time - 0.1 for latency

    //     //private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
    //     //https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java

    //   }

    // }




  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
