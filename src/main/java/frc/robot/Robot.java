// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.odometry.Limelight;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        Timer.delay(4);
        Limelight.blink();

        m_robotContainer = new RobotContainer();
        m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
        // for (int port = 5800; port <= 5805; port++) {
        //    PortForwarder.add(port, "limelight.local", port);
        // }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Limelight.updatePose(m_robotContainer.drivetrain);

        // Checks if robot is close to speaker prior to limelight correction (avoids limelight noise)

        // if (Limelight.limelightResults != null && Limelight.llPose != null && Limelight.limelightResults.valid) {
        //     if (Limelight.limelightResults.targets_Fiducials.length > 1 && (
        //         m_robotContainer.drivetrain.getState().Pose.getX() < 4 || m_robotContainer.drivetrain.getState().Pose.getX() > 12.55
        //     )) {
        //         m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 999999999));
        //         m_robotContainer.drivetrain.addVisionMeasurement(Limelight.llPose, Timer.getFPGATimestamp() - (
        //                 Limelight.limelightResults.latency_capture
        //                         + Limelight.limelightResults.latency_jsonParse
        //                         + Limelight.limelightResults.latency_pipeline) / 1000
        //         );
        //     }
        // }


        //SmartDashboard.putNumber("AA_Angle off X axis", m_robotContainer.drivetrain.getState().Pose.getTranslation().getAngle().getDegrees());
        // if (false) {

        // var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
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
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        m_robotContainer.flipper.init();
        m_robotContainer.pivot.init();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {

        m_robotContainer.flipper.init();
        m_robotContainer.pivot.init();


        if (DriverStation.getAlliance().get() == Alliance.Red) {
            System.out.println("Setting direction for red alliance");
            m_robotContainer.drivetrain.setAllianceDirection(Alliance.Red);
        } else {
            System.out.println("Setting direction for blue alliance");
            m_robotContainer.drivetrain.setAllianceDirection(Alliance.Blue);
        }

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    //(lastResult.latency_capture + lastResult.latency_jsonParse + lastResult.latency_pipeline)/1000);
    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
