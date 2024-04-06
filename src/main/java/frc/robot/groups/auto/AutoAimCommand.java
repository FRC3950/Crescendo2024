package frc.robot.groups.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.NoteKinematics;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class AutoAimCommand extends SequentialCommandGroup {
  public AutoAimCommand(Pivot pivot, Intake intake, Shooter shooter, DoubleSupplier angle) {
    addCommands(
      Commands.parallel(
        pivot.setAngleCommand(angle),
        shooter.applyVelocitiesCommand() // Finishes when at velocity   
      )
    );
    addRequirements(pivot, intake, shooter);
  }
}
