package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import lib.odometry.ScoringKinematics;

import java.util.function.DoubleSupplier;

public class ScoreCommands {

    private final Intake intake;
    private final Shooter shooter;
    private final Flipper flipper;
    private final Pivot pivot;
    private final Swerve swerve;

    public ScoreCommands(Intake intake, Shooter shooter, Flipper flipper, Pivot pivot, Swerve swerve) {
        this.intake = intake;
        this.shooter = shooter;
        this.flipper = flipper;
        this.pivot = pivot;
        this.swerve = swerve;
    }

    public Command indexShootCommand() {
        return Commands.sequence(
            shooter.applyShootState(),
            intake.indexCommand(),
            Commands.waitSeconds(0.2),
            intake.stopCommand()
        );
    }

    public Command aimPivotCommand(){
        return Commands.parallel(
                Commands.runOnce(() -> swerve.isLockedRotational = true),
                pivot.continuousAngleCommand(() -> ScoringKinematics.getTargetPivot(
                    () -> ScoringKinematics.getAllianceSpeakerDistance(swerve)
                )),
                shooter.applyShootState()
        );
    }

    public Command lobPivotCommand() {
        return Commands.parallel(
                Commands.runOnce(() -> swerve.isLockedRotational = true),
                pivot.continuousAngleCommand(() -> ScoringKinematics.getLobPivot(
                        () -> ScoringKinematics.getAllianceSpeakerDistance(swerve)
                )),
                shooter.applyLobState(() -> ScoringKinematics.getAllianceSpeakerDistance(swerve))
        );
    }

    public Command ampScoreCommand(DoubleSupplier angle) {
        return Commands.parallel(
            pivot.ampCommand(),
            flipper.ampCommand(pivot)
        );
    }
}
