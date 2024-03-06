// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.paths.OdometryStageAlign;
import frc.robot.commands.paths.TurnToGoal;
import frc.robot.commands.teleop.IntakeCommand;
import frc.robot.commands.teleop.ShootCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Telemetry;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  Alliance my_alliance; 


  private double MaxSpeed = 5; // 6 meters per second desired top speed *t3x*
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //Joystick and drivetrain
  private final CommandXboxController joystick = new CommandXboxController(0); 
  private final CommandXboxController SecondJoystick = new CommandXboxController(1);
  public final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final Pivot pivot = new Pivot(); // My pivot subsystem

  Pose2d redSpeaker = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(180));
  Pose2d blueSpeaker = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));
  Pose2d inFrontOfSpeaker = new Pose2d(2,5.55, Rotation2d.fromDegrees(0));

  private final ShootCommand shoot = new ShootCommand();
  private final IntakeCommand intakeIn = new IntakeCommand(true);
  private final IntakeCommand outtake = new IntakeCommand(false);

  

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
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(drivetrain.getRotationalSpeed(() -> -joystick.getRightX()) * MaxAngularRate)                                                                                // X (left)
    ).ignoringDisable(true));
    // Toggle April-Tag Lock on (Robot with drive angled at tag)
        PathPlannerPath midNoteShootPos = PathPlannerPath.fromPathFile("driveToNoteShot");

    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> { drivetrain.isLockedRotational = !drivetrain.isLockedRotational;    }));
    joystick.rightBumper().onTrue(AutoBuilder.followPath(midNoteShootPos)
                .onlyWhile(() -> Math.abs(joystick.getLeftY()) < 0.5 && Math.abs(joystick.getLeftX()) < 0.5));
                

    joystick.leftTrigger(0.5).onTrue(new AimAndShootCommand(pivot, 17));
    joystick.rightTrigger(0.5).onTrue(new AimAndShootCommand(pivot, 26));

    

    // reset the field-centric heading 
    joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //test reset Pose
    //TODO make it conditional
    joystick.b().onTrue(
      new InstantCommand()
    );

    // AutoBuilder.pathfindToPose(inFrontOfSpeaker, 
    
    //   new PathConstraints(
    //     4.0, 4.0,
    //     Units.degreesToRadians(540), Units.degreesToRadians(720)),
    //     0,
    //     0);

    // update Pose with Vision
    joystick.a().onTrue(drivetrain.runOnce(drivetrain::applyVisiontoPose));



    
    SecondJoystick.x().whileTrue(shoot)
    .onFalse(Commands.runOnce(Intake.getInstance()::stop,Intake.getInstance()));
    
    SecondJoystick.a().whileTrue(   
      
      Commands.runEnd(Intake.getInstance()::run, Intake.getInstance()::stop, 
      Intake.getInstance()).until(()-> Intake.getInstance().noteIsIndexed()));
      
      // .and( ()-> !Intake.getInstance().noteIsIndexed());


    
    
    SecondJoystick.b().whileTrue(outtake);

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

      // SmartDashboard.putData("shootHub",new AimAndShootCommand(pivot, 17));
      //   SmartDashboard.putData("Note",new AimAndShootCommand(pivot, 26));

    NamedCommands.registerCommand("shootHub", new AimAndShootCommand(pivot, 17));
    NamedCommands.registerCommand("shootNote", new AimAndShootCommand(pivot, 26));
    NamedCommands.registerCommand("intakeOn", new IntakeCommand(true));

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

    SmartDashboard.putData("turnToShoot", new TurnToGoal(drivetrain).onlyWhile(() -> Math.abs(joystick.getLeftY()) < 0.5 && Math.abs(joystick.getLeftX()) < 0.5));
        
        
    //SmartDashboard.putString("alliance", DriverStation.getAlliance().get().toString());

    my_alliance = DriverStation.getAlliance().get();
    

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

    SmartDashboard.putData("shootHub",new AimAndShootCommand(pivot, 17));
        SmartDashboard.putData("Note",new AimAndShootCommand(pivot, 26));

        SmartDashboard.putData("aaaa", new RunCommand(()->pivot.adjustAngle(17), pivot));


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
