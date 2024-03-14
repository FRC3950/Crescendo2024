package frc.robot.xbox;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public enum Controller {
    DRIVER(new CommandXboxController(0)),
    MANIPULATOR(new CommandXboxController(1));

    public final CommandXboxController controller;

    Controller(CommandXboxController id) {
        this.controller = id;
    }
}
