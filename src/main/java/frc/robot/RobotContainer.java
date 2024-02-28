// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.pathCommands.alignWithStage_Odometry;
import frc.robot.pathCommands.turnToShootGoal;
import frc.robot.subsystems.Pivot_Subsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  Alliance my_alliance;




  private double MaxSpeed = 5; // 6 meters per second desired top speed *t3x*
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

//Joystick and drivetrain

  private final CommandXboxController joystick = new CommandXboxController(0); 
  public final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final Pivot_Subsystem pivot = new Pivot_Subsystem(); // My pivot subsystem

    Pose2d redSpeaker = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(180));
     Pose2d blueSpeaker = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));

//Field Centric Request - field-centric in open loop

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.10).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric
                                                               // driving in open loop

//Robot Centric Request   

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


                                                                                  ////// Break and point are not used in the code, but left for reference /////////////
                                                                                      // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
                                                                                      // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
                                                                                  //////////////////////////////////////////////////////////////////////////////////////

//Creates our Telemetry object
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.getRotationalSpeed(() -> -joystick.getRightX()) * MaxAngularRate) // Drive
                                                                                                             // counterclockwise
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // X (left)
        ).ignoringDisable(true));

    // Toggle April-Tag Lock on (Robot with drive angled at tag)
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> { drivetrain.isLockedRotational = !drivetrain.isLockedRotational;    }));

    // reset the field-centric heading 
    joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //test reset Pose
   joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(redSpeaker)));

    // update Pose with Vision
    joystick.a().onTrue(drivetrain.runOnce(drivetrain::applyVisiontoPose));


    // joystick.leftTrigger(0.2).whileTrue(intake.runIntake(()->joystick.getLeftTriggerAxis()));


    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    joystick.pov(90).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));
    joystick.pov(270).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));

    // joystick.x().whileTrue(new PathPlannerAuto(("test")));
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
      // joystick.b().whileTrue(drivetrain
      //   .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

   
  
  }


  

  public RobotContainer() {

          if (Utils.isSimulation()) {
    drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    Rotation2d.fromDegrees(90)));
          }

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)));


    // Constructs AutoBuilder (SendableChooser<Command>):
    autoChooser = AutoBuilder.buildAutoChooser("andy");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PathPlannerPath upperStage = PathPlannerPath.fromPathFile("UpperStage");
    PathPlannerPath lowerStage = PathPlannerPath.fromPathFile("LowerStage");
    PathPlannerPath amp = PathPlannerPath.fromPathFile("Amp");
    // SmartDashboard.putData("goStage", new alignWithStage_Odometry(new
    // Pose2d()).onlyWhile(()-> Math.abs(joystick.getLeftY()) > 0.5));
    // y greater than 4
    SmartDashboard.putData("goStage",
        Commands.either(
            AutoBuilder.followPath(upperStage)
                .onlyWhile(() -> Math.abs(joystick.getLeftY()) < 0.5 && Math.abs(joystick.getLeftX()) < 0.5),
            AutoBuilder.followPath(lowerStage)
                .onlyWhile(() -> Math.abs(joystick.getLeftY()) < 0.5 && Math.abs(joystick.getLeftX()) < 0.5),
            () -> drivetrain.getState().Pose.getY() > 4));

    SmartDashboard.putData("amp", AutoBuilder.followPath(amp).onlyWhile(() -> Math.abs(joystick.getLeftY()) < 0.5 && Math.abs(joystick.getLeftX()) < 0.5));

    SmartDashboard.putData("turnToShoot", new turnToShootGoal(drivetrain).onlyWhile(() -> Math.abs(joystick.getLeftY()) < 0.5 && Math.abs(joystick.getLeftX()) < 0.5));

  
 

          
        
        
    //SmartDashboard.putString("alliance", DriverStation.getAlliance().get().toString());

    if(Utils.isSimulation()){
      var simStation = DriverStationSim.getAllianceStationId().toString();
      if(simStation.equals("Red1") || simStation.equals("Red2") || simStation.equals("Red3")){
      my_alliance = DriverStation.Alliance.Red;
      }
      else{
         my_alliance = DriverStation.Alliance.Blue;}
        
    }
    else{
      my_alliance = DriverStation.getAlliance().get();
    }
    

    SmartDashboard.putData("toggleAutoAngle",
    pivot.autoAngleCommand(
      
    my_alliance == Alliance.Blue ? 
    () -> drivetrain.getState().Pose.getTranslation().getDistance(blueSpeaker.getTranslation()) :
    () -> drivetrain.getState().Pose.getTranslation().getDistance(redSpeaker.getTranslation())
        ).onlyIf(()->DriverStation.getAlliance().isPresent()));


    SmartDashboard.putData("PrintAngle", new RunCommand(() -> {
      SmartDashboard.putNumber("Angle Offset From X Axis on Blue Speaker", 
      
      Math.atan(

        drivetrain.getState().Pose.relativeTo(blueSpeaker).getY()/drivetrain.getState().Pose.relativeTo(blueSpeaker).getX()) * 180 / Math.PI);

      

    }));

    // why does addVisionMeasurement not work?
    // also do speak center path

    // then shooter and angle subsystem

    // then intake and conveyor subsystem

    // SmartDashboard.putData("addVisionMeasurment", new InstantCommand(
    // () -> {
    // drivetrain.addVisionMeasurement(new Pose2d(7, 7, Rotation2d.fromDegrees(90)),
    // Timer.getFPGATimestamp());
    // //System.out.println(drivetrain.getState().Pose);
    // }
    // ));

    configureBindings();
  }

  private Alliance getAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){}
    return DriverStation.getAlliance().get();
  }


  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
}
