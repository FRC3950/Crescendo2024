// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.odometry.Limelight;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private static boolean isVisionTrackingEnabled = true;

    @Override
    public void robotInit() {

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

        SmartDashboard.putNumber("BridgwoodGoesWEEEEEE", m_robotContainer.drivetrain.getState().Pose.getTranslation().getDistance(new Translation2d(0,5.55)));
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 6));

        Limelight.updateHeadingMt1(m_robotContainer.drivetrain);

    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 6));

        Limelight.updateHeadingMt1(m_robotContainer.drivetrain);

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        m_robotContainer.flipper.init();
        m_robotContainer.pivot.init();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        if(isVisionTrackingEnabled){
                Limelight.updatePose(m_robotContainer.drivetrain);

        }
    }

    public static void setVisionTrackingEnabled(boolean enabled){
        isVisionTrackingEnabled = enabled;

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
                Limelight.updatePose(m_robotContainer.drivetrain);


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
