// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class AmpScoreCommand extends SequentialCommandGroup {
    /**
     * Creates a new AmpScoreCommand.
     */
    public AmpScoreCommand(Pivot pivot, Flipper flipper, Shooter shooter, DoubleSupplier pivotPos, DoubleSupplier flipperPos) {
        addCommands(
                shooter.stopCommand(),
                pivot.setAngleInstantCommand(pivotPos),
                Commands.waitSeconds(0.6),
                flipper.ampCommand(flipperPos)
        ); // Stow command runs on button false (See RobotContainer)

        addRequirements(pivot, flipper, shooter);
    }
}
