// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.groups.shoot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class LobShootCommand extends SequentialCommandGroup {
  public LobShootCommand(Shooter shooter, Swerve drivetrain, Pivot pivot, Intake intake, Supplier<DriverStation.Alliance> alliance, Supplier<Translation2d> blueSpeaker, Supplier<Translation2d> redSpeaker) {

    final boolean isBlue = alliance.get() == DriverStation.Alliance.Blue;
    final DoubleSupplier distance = () -> drivetrain.getState().Pose.getTranslation().getDistance((isBlue) ? blueSpeaker.get() : redSpeaker.get());

    addCommands(
        Commands.parallel(
          shooter.applyLobVelocity(distance),
          pivot.lobAngleCommand(distance),
          Commands.runOnce(() -> drivetrain.isLockedRotational = true),
          Commands.runOnce(() -> shooter.isLobbing = true)
        )
    );
  }
}
