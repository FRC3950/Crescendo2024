package frc.robot.groups;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class VisionAutoAimCommand extends ParallelCommandGroup {
    public VisionAutoAimCommand(Pivot pivot, Swerve drivetrain, Supplier<DriverStation.Alliance> alliance, Supplier<Translation2d> blueSpeaker, Supplier<Translation2d> redSpeaker) {

        final boolean isBlue = alliance.get() == DriverStation.Alliance.Blue;

        addCommands(
            pivot.commands.autoAngleCommand(() -> drivetrain.getState().Pose.getTranslation().getDistance(
                    ((isBlue) ? blueSpeaker.get() : redSpeaker.get())
            )),
            Commands.runOnce(() -> drivetrain.isLockedRotational = true)
        );
    }
}