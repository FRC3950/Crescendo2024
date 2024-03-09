// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleop.ShootCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShootCommand extends SequentialCommandGroup {
  /** Creates a new AimAndShootCommand. */
  public AimAndShootCommand(Pivot pivot, double angle) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      pivot.run(()->pivot.adjustAngle(angle)).until(()-> pivot.isAtAngle(angle)),

      new ParallelCommandGroup(
        pivot.run(()->pivot.adjustAngle(angle)),
        new ShootCommand()

    ).withTimeout(1.25),
      new InstantCommand(()->Intake.getInstance().stop()),
      pivot.runOnce(()->pivot.adjustAngle(0)).until(()-> pivot.isAtAngle(0))
    );
  }
}
