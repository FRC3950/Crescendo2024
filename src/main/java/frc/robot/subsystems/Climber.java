package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.supersystems.PositionController;
import frc.robot.supersystems.TargetPosition;

public class Climber extends PositionController {

    public Climber() {
        super(new TargetPosition(
                new TalonFX(Constants.Climber.motor1Id),
                new TalonFX(Constants.Climber.motor2Id),
                () -> 0,
                Constants.Climber.kP,
                Constants.Climber.kV,
                Constants.Climber.kG,
                false
        ));
    }

    public Command stowCommand() {
        return Commands.runOnce(() -> setPosition(() -> 0));
    }

    public Command raiseCommand() {
        return Commands.runOnce(() -> setPosition(() -> 10));
    }
}
