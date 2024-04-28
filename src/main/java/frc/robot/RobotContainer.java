// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ManipulateCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.constants.TunerConstants;
import frc.robot.commands.auto.AutoAimPathCommand;
import frc.robot.commands.auto.AutoAimShootCommand;
import frc.robot.subsystems.*;
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

    // Subsystems
    public final Pivot pivot = new Pivot();
    public final Flipper flipper = new Flipper();

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();

    private final ManipulateCommands manipulateCommands = new ManipulateCommands(intake, pivot, flipper);
    private final ScoreCommands scoreCommands = new ScoreCommands(intake, shooter, flipper, pivot, drivetrain);

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
                drivetrain.applyRequest(() -> drive.withVelocityX(-Controller.DRIVER.getLeftY() * MaxSpeed)
                        .withVelocityY(-Controller.DRIVER.getLeftX() * MaxSpeed)
                        .withRotationalRate(drivetrain.getRotationalSpeed(() -> -Controller.DRIVER.getRightX()) * MaxAngularRate)                                                                                // X (left)
                ).ignoringDisable(true)
        );

        climber.setDefaultCommand(climber.climbCommand(Controller.MANIPULATOR::getRightY));


        ControlScheme.RESET_HEADING.button.onTrue(Commands.runOnce(drivetrain::seedFieldRelative));
        PathPlannerPath amp_PathPlanner = PathPlannerPath.fromPathFile("Amp");
        PathPlannerPath speaker_PathPlanner = PathPlannerPath.fromPathFile("Speaker");

        PathPlannerPath topStage_PathPlannerPath = PathPlannerPath.fromPathFile("TopStage");
        PathPlannerPath bottomStage_PathPlannerPath = PathPlannerPath.fromPathFile("BottomStage");

        ControlScheme.PATH_AMP.button.onTrue(AutoBuilder.followPath(amp_PathPlanner)
        .onlyWhile(() -> Math.abs(Controller.DRIVER.getLeftY()) < 0.5 && Math.abs(Controller.DRIVER.getLeftX()) < 0.5));

        ControlScheme.PATH_SPEAKER.button.onTrue(AutoBuilder.followPath(speaker_PathPlanner)
        .onlyWhile(() -> Math.abs(Controller.DRIVER.getLeftY()) < 0.5 && Math.abs(Controller.DRIVER.getLeftX()) < 0.5));

//        ControlScheme.PATH_STAGE.button.onTrue(
//                Commands.either(
//                        AutoBuilder.followPath(bottomStage_PathPlannerPath)
//                        .onlyWhile(() -> Math.abs(Controller.DRIVER.getLeftY()) < 0.5 && Math.abs(Controller.DRIVER.getLeftX()) < 0.5),
//                        AutoBuilder.followPath(topStage_PathPlannerPath)
//                         .onlyWhile(() -> Math.abs(Controller.DRIVER.getLeftY()) < 0.5 && Math.abs(Controller.DRIVER.getLeftX()) < 0.5),
//                        () -> drivetrain.getState().Pose.getY() < 4.44));


        // Manipulator controls
        ControlScheme.SHOOT_SPEAKER.button.whileTrue(scoreCommands.speakerPivotCommand()).onFalse(manipulateCommands.stowCommand());

        ControlScheme.SHOOT.button.whileTrue(shooter.applyShootStateCommand()).onFalse(shooter.idleCommand());

        ControlScheme.SHOOT_LOB.button.whileTrue(scoreCommands.lobPivotCommand()).onFalse(
                Commands.parallel(
                        Commands.runOnce(() -> drivetrain.isLockedRotational = false),
                        manipulateCommands.stowCommand()
                )
        );

        ControlScheme.SCORE_AMP.button.whileTrue(scoreCommands.ampPivotCommand()).onFalse(manipulateCommands.stowCommand());

        ControlScheme.AIM_AUTO.button.whileTrue(scoreCommands.aimCommand()).onFalse(
                Commands.parallel(
                        Commands.runOnce(() -> drivetrain.isLockedRotational = false),
                        manipulateCommands.stowCommand()
                ));

        // SmartDashboard.putData("AutoAim", autoShootCommand);
        // SmartDashboard.putData("RotationOff", Commands.runOnce(() -> drivetrain.isLockedRotational = false));

        ControlScheme.INTAKE.button.whileTrue(manipulateCommands.intakeCommand()).onFalse(manipulateCommands.stowCommand());

        ControlScheme.FORCE_STOW.button.whileTrue(pivot.forceStowCommand());

        ControlScheme.OUTTAKE.button.whileTrue(intake.outtakeCommand(pivot));

        Controller.DRIVER.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        Controller.DRIVER.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        Controller.DRIVER.pov(90).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));
        Controller.DRIVER.pov(270).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));

        // Controller.DRIVER.controller.x().whileTrue(new PathPlannerAuto(("test")));
        // Controller.DRIVER.controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // Controller.DRIVER.controller.b().whileTrue(drivetrain
        //   .applyRequest(() -> point.withModuleDirection(new Rotation2d(-Controller.DRIVER.controller.getLeftY(), -Controller.DRIVER.controller.getLeftX()))));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {

        //For Sim testing fluid Auto
        //  SmartDashboard.putBoolean("noteIndexed", true);
        //  SmartDashboard.putData(new AutoAimPathCommand(pivot, intake, shooter, drivetrain));
        //  SmartDashboard.putData("forceShoot", intake.indexCommand().withTimeout(1));


        NamedCommands.registerCommand("autoAim", new AutoAimPathCommand(pivot, intake, shooter, drivetrain));
        NamedCommands.registerCommand("simpleAimAndShoot", new AutoAimShootCommand(pivot, intake, shooter, drivetrain));

        NamedCommands.registerCommand("intakeOn", intake.intakeCommand());
        NamedCommands.registerCommand("intakeOff", intake.stopCommand());

        NamedCommands.registerCommand("shootHub", new AutoAimShootCommand(pivot, intake, shooter, drivetrain, ()-> 0.1));

        NamedCommands.registerCommand("visionOn", new InstantCommand(()->Robot.setVisionTrackingEnabled(true)));

        NamedCommands.registerCommand("forceShoot", intake.indexCommand());

        NamedCommands.registerCommand("stow", pivot.stowInstantCommand());


        // Constructs AutoBuilder (SendableChooser<Command>):
        autoChooser = AutoBuilder.buildAutoChooser("1pc");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // PathPlannerPath upperStage = PathPlannerPath.fromPathFile("UpperStage");
        // PathPlannerPath lowerStage = PathPlannerPath.fromPathFile("LowerStage");
        // PathPlannerPath amp = PathPlannerPath.fromPathFile("Amp");

        // my_alliance = DriverStation.getAlliance().get();

        configureBindings();
    }

    private Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        }
        return Alliance.Red;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
