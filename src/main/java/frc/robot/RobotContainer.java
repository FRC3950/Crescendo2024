// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.groups.AimShootCommand;
import frc.robot.groups.VisionAutoAimCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.controller.ControlScheme;
import frc.robot.controller.Controller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Telemetry;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  Alliance my_alliance;


  private final double MaxSpeed = 4.75; // 6 meters per second desired top speed *t3x*  //was 5 before
  private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //Joystick and drivetrain
  public final Swerve drivetrain = TunerConstants.DriveTrain;

  Pose2d redSpeaker = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(0));
  Pose2d blueSpeaker = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));
  Pose2d inFrontOfSpeaker = new Pose2d(2,5.55, Rotation2d.fromDegrees(0));

  // Subsystems
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  public final Pivot pivot = new Pivot();

  private final Command autoAimCommand = new VisionAutoAimCommand(
          pivot, drivetrain,
          this::getAlliance,
          blueSpeaker::getTranslation,
          redSpeaker::getTranslation
  );

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

    drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> drive.withVelocityX(-Controller.DRIVER.controller.getLeftY() * MaxSpeed)
        .withVelocityY(-Controller.DRIVER.controller.getLeftX() * MaxSpeed)
        .withRotationalRate(drivetrain.getRotationalSpeed(() -> -Controller.DRIVER.controller.getRightX()) * MaxAngularRate)                                                                                // X (left)
    ).ignoringDisable(true));

    PathPlannerPath midNoteShootPos = PathPlannerPath.fromPathFile("driveToNoteShot");

    Controller.DRIVER.controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.isLockedRotational = !drivetrain.isLockedRotational));
    Controller.DRIVER.controller.rightBumper().onTrue(
    AutoBuilder.followPath(midNoteShootPos)
                .onlyWhile(() -> Math.abs(Controller.DRIVER.controller.getLeftY()) < 0.5 && Math.abs(Controller.DRIVER.controller.getLeftX()) < 0.5) // Cancels on driver input
    );
    ControlScheme.RESET_HEADING.button.onTrue(Commands.runOnce(drivetrain::seedFieldRelative));

    // Manipulator controls
    ControlScheme.SHOOT_SPEAKER.button.whileTrue(new AimShootCommand(pivot, intake, shooter, () -> 16));
    ControlScheme.SHOOT_STAGE.button.whileTrue(new AimShootCommand(pivot, intake, shooter, () -> 27));

    ControlScheme.AIM_AUTO.button.whileTrue(autoAimCommand)
                    .onFalse(Commands.sequence(
                            Commands.runOnce(() -> drivetrain.isLockedRotational = false),
                            pivot.commands.stowCommand()
                    ));

    ControlScheme.INTAKE.button.whileTrue(intake.commands.intakeCommand());
    ControlScheme.OUTTAKE.button.whileTrue(intake.commands.outtakeCommand());
    ControlScheme.SHOOT.button.whileTrue(shooter.commands.shoot(intake));

    Controller.DRIVER.controller.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    Controller.DRIVER.controller.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    Controller.DRIVER.controller.pov(90).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));
    Controller.DRIVER.controller.pov(270).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));

    // Controller.DRIVER.controller.x().whileTrue(new PathPlannerAuto(("test")));
    // Controller.DRIVER.controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // Controller.DRIVER.controller.b().whileTrue(drivetrain
    //   .applyRequest(() -> point.withModuleDirection(new Rotation2d(-Controller.DRIVER.controller.getLeftY(), -Controller.DRIVER.controller.getLeftX()))));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    SmartDashboard.putData(Commands.runOnce(intake.commands::intakeCommand));

    NamedCommands.registerCommand("shootHub", new AimShootCommand(pivot, intake, shooter, () -> 17));
    NamedCommands.registerCommand("shootNote", new AimShootCommand(pivot, intake, shooter, () -> 26));
    NamedCommands.registerCommand("intakeOn", intake.commands.intakeCommand().withTimeout(1.5));
    NamedCommands.registerCommand("intakeOff", intake.commands.stopCommand());
    // Constructs AutoBuilder (SendableChooser<Command>):
    autoChooser = AutoBuilder.buildAutoChooser("andy");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PathPlannerPath upperStage = PathPlannerPath.fromPathFile("UpperStage");
    PathPlannerPath lowerStage = PathPlannerPath.fromPathFile("LowerStage");
    PathPlannerPath amp = PathPlannerPath.fromPathFile("Amp");

    my_alliance = DriverStation.getAlliance().get();


    SmartDashboard.putData("toggleAutoAngle",
    pivot.commands.autoAngleCommand(
            my_alliance == Alliance.Blue ?
        () -> drivetrain.getState().Pose.getTranslation().getDistance(blueSpeaker.getTranslation()) :
        () -> drivetrain.getState().Pose.getTranslation().getDistance(redSpeaker.getTranslation()))
            .onlyIf(()->DriverStation.getAlliance().isPresent())
    );

    configureBindings();
  }

  private Alliance getAlliance() {
    return DriverStation.getAlliance().get();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
