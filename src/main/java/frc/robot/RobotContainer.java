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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.groups.AimCommand;
import frc.robot.groups.AimShootCommand;
import frc.robot.groups.AmpScoreCommand;
import frc.robot.groups.IntakeCommand;
import frc.robot.groups.VisionShootCommand;
import frc.robot.groups.auto.AutoAimShootCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.xbox.ControlScheme;
import frc.robot.xbox.Controller;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  public static Alliance my_alliance;

  public final NetworkTableEntry velocityX = NetworkTableInstance.getDefault().getEntry("Drive").getTopic().getInstance().getEntry("Velocity X");
  public final NetworkTableEntry velocity = NetworkTableInstance.getDefault().getEntry("Drive").getTopic().getInstance().getEntry("Velocity Y");


  private final double MaxSpeed = 4.3; // 6 meters per second desired top speed *t3x*  //was 5 before
  private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //Joystick and drivetrain
  public final Swerve drivetrain = TunerConstants.DriveTrain;

  Pose2d redSpeaker = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(0));
  Pose2d blueSpeaker = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));
  Pose2d inFrontOfSpeaker = new Pose2d(2,5.55, Rotation2d.fromDegrees(0));

  // Subsystems
  public final Pivot pivot = new Pivot();
  public final Flipper flipper = new Flipper();

  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();


  private final Command autoShootCommand = new VisionShootCommand(
          pivot, drivetrain, shooter, intake,
          this::getAlliance,
          blueSpeaker::getTranslation,
          redSpeaker::getTranslation
  );

  // private final StowCommand stowCommand = new StowCommand(pivot, intake, shooter, flipper);
  // This causes WPILib to crash for some reason whenever it is ran

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
      ).ignoringDisable(true)
    );

    

    climber.setDefaultCommand(climber.climbCommand(() -> Controller.MANIPULATOR.controller.getRightY()));

   // PathPlannerPath midNoteShootPos = PathPlannerPath.fromPathFile("driveToNoteShot");

    // Controller.DRIVER.controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.isLockedRotational = !drivetrain.isLockedRotational));
    // Controller.DRIVER.controller.rightBumper().onTrue(
    //   AutoBuilder.followPath(midNoteShootPos)
    //               .onlyWhile(() -> Math.abs(Controller.DRIVER.controller.getLeftY()) < 0.5 && Math.abs(Controller.DRIVER.controller.getLeftX()) < 0.5) // Cancels on driver input
    // );

    ControlScheme.RESET_HEADING.button.onTrue(Commands.runOnce(drivetrain::seedFieldRelative));

    // Manipulator controls
    ControlScheme.SHOOT_SPEAKER.button.whileTrue(
      new AimCommand(pivot, shooter, () -> 0.1)
    ).onFalse(Commands.parallel(
      pivot.stowCommand(),
      flipper.stowCommand(),
      shooter.idleCommand(),
      intake.stopCommand()
    ));

    ControlScheme.SHOOT.button.whileTrue(shooter.shootCommand(intake));

    ControlScheme.SCORE_AMP.button.whileTrue(new AmpScoreCommand(pivot, flipper, ()-> 0.3, Constants.Flipper.ampPosition))
      .onFalse(Commands.parallel(
        pivot.stowCommand(),
        flipper.stowCommand(),
        shooter.idleCommand(),
        intake.stopCommand()
      ));

    /*ControlScheme.SCORE_TRAP.button.toggleOnTrue(Commands.runEnd(
      new AmpScoreCommand(pivot, flipper, ()-> 0.45, Constants.Flipper.trapPosition),
      Commands.parallel(
        pivot.stowCommand(),
        flipper.stowCommand(),
        shooter.idleCommand(),
        intake.stopCommand())

    ));*/

    ControlScheme.AIM_AUTO.button.whileTrue(autoShootCommand)
      .onFalse(Commands.parallel(
              Commands.runOnce(() -> drivetrain.isLockedRotational = false),
              Commands.parallel(
                pivot.stowCommand(),
                flipper.stowCommand(),
                shooter.idleCommand(),
                intake.stopCommand()
              )
      ));
      SmartDashboard.putData("AutoAim", autoShootCommand);
            SmartDashboard.putData("RotationOff", Commands.runOnce(() -> drivetrain.isLockedRotational = false));


    ControlScheme.INTAKE.button.whileTrue(new IntakeCommand(pivot, intake))
      .onFalse(pivot.stowCommand());

    ControlScheme.FORCE_STOW.button.whileTrue(pivot.forceStowCommand());

    ControlScheme.OUTTAKE.button.whileTrue(intake.outtakeCommand());
    ControlScheme.AMP_OUTTAKE.button.whileTrue(intake.ampOuttakeCommand());

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
    SmartDashboard.putData(Commands.runOnce(() -> intake.intakeCommand()));

    NamedCommands.registerCommand("shootHub", new AutoAimShootCommand(pivot, intake, shooter, drivetrain, this::getAlliance, redSpeaker::getTranslation, blueSpeaker::getTranslation));
    NamedCommands.registerCommand("shootNote", new AutoAimShootCommand(pivot, intake, shooter, () -> 0));
    NamedCommands.registerCommand("shootFarNote", new AutoAimShootCommand(pivot, intake, shooter, () -> 0));
    NamedCommands.registerCommand("shootReallyFar", new AutoAimShootCommand(pivot, intake, shooter, () -> 0));
    //  .onFalse(Commands.parallel(
    //           Commands.runOnce(() -> drivetrain.isLockedRotational = false),
    //           Commands.parallel(
    //             pivot.stowCommand(),
    //             flipper.stowCommand(),
    //             shooter.idleCommand(),
    //             intake.stopCommand()
    //           )
    //   ));


    NamedCommands.registerCommand("intakeOn", intake.intakeCommand());
    NamedCommands.registerCommand("intakeOff", intake.stopCommand());

    // Constructs AutoBuilder (SendableChooser<Command>):
    autoChooser = AutoBuilder.buildAutoChooser("1pc");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // PathPlannerPath upperStage = PathPlannerPath.fromPathFile("UpperStage");
    // PathPlannerPath lowerStage = PathPlannerPath.fromPathFile("LowerStage");
    // PathPlannerPath amp = PathPlannerPath.fromPathFile("Amp");

    my_alliance = DriverStation.getAlliance().get();

    // SmartDashboard.putData("toggleAutoAngle",
    // pivot.autoAngleCommand(
    //         my_alliance == Alliance.Blue ?
    //     () -> drivetrain.getState().Pose.getTranslation().getDistance(blueSpeaker.getTranslation()) :
    //     () -> drivetrain.getState().Pose.getTranslation().getDistance(redSpeaker.getTranslation()))
    //         .onlyIf(()->DriverStation.getAlliance().isPresent())
    // );

    configureBindings();
  }

  private Alliance getAlliance() {
    return DriverStation.getAlliance().get();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
