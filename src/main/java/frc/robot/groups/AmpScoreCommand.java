// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class AmpScoreCommand extends ParallelCommandGroup {

    public AmpScoreCommand(Pivot pivot, Flipper flipper, Shooter shooter, DoubleSupplier pivotPos, DoubleSupplier flipperPos) {
        addCommands(
                shooter.stopCommand(),
                pivot.setAngleInstantCommand(pivotPos),
                flipper.ampCommand(pivot)
        ); // Stow command runs on button false (See RobotContainer)

        addRequirements(pivot, flipper, shooter);
    }
}
