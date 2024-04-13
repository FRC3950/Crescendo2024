package frc.robot.groups.shoot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class VisionShootCommand extends SequentialCommandGroup {
    public VisionShootCommand(Pivot pivot, Swerve drivetrain, Shooter shooter, Supplier<DriverStation.Alliance> alliance, Supplier<Translation2d> blueSpeaker, Supplier<Translation2d> redSpeaker) {

        final boolean isBlue = alliance.get() == DriverStation.Alliance.Blue;

        addCommands(
                Commands.parallel(
                        shooter.applyVelocitiesCommand(),
                        pivot.autoAngleCommand(() -> drivetrain.getState().Pose.getTranslation().getDistance(
                                ((isBlue) ? blueSpeaker.get() : redSpeaker.get())
                        )),
                        Commands.runOnce(() -> drivetrain.isLockedRotational = true)
                )
        ); // See RobotContainer for interrupt behavior
    }
}