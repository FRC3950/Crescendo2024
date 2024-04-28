package frc.robot.xbox;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface Controller {
    CommandXboxController DRIVER = new CommandXboxController(0);
    CommandXboxController MANIPULATOR = new CommandXboxController(1);
}
