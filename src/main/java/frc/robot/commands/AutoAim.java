// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleop.ShootCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAim extends SequentialCommandGroup {
  /** Creates a new AimAndShootCommand. */
  public AutoAim(Pivot pivot, DoubleSupplier ourDistanceFromShot) {

    // -1.808x^2 +14.70x +1.332

    var distance = ourDistanceFromShot.getAsDouble();
    var angle = -1.808 * Math.pow(distance, 2) + 14.7 * distance + 1.332;


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    Commands.runEnd(      
      ()->pivot.adjustAngle(angle),
       ()->pivot.adjustAngle(0),
pivot)

     
    );
  }
}
