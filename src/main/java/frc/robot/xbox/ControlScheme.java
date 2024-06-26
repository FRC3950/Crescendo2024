package frc.robot.xbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public enum ControlScheme {
    // Manipulator
    SHOOT_SPEAKER(Controller.MANIPULATOR.controller.leftBumper()),
    FORCE_STOW(Controller.MANIPULATOR.controller.rightBumper()),
    SHOOT(Controller.MANIPULATOR.controller.rightTrigger(0.5)),
    SHOOT_LOB(Controller.MANIPULATOR.controller.pov(0)),
    SCORE_AMP(Controller.MANIPULATOR.controller.x()),
    SCORE_TRAP(Controller.MANIPULATOR.controller.pov(180)),

    AIM_AUTO(Controller.MANIPULATOR.controller.leftTrigger(0.5)),

    INTAKE(Controller.MANIPULATOR.controller.a()),
    OUTTAKE(Controller.MANIPULATOR.controller.b()),

    AMP_OUTTAKE(Controller.MANIPULATOR.controller.y()),

    // Driver
    RESET_HEADING(Controller.DRIVER.controller.y()),
    PATH_AMP(Controller.DRIVER.controller.leftBumper()),
    PATH_SPEAKER(Controller.DRIVER.controller.rightBumper()),
    PATH_STAGE(Controller.DRIVER.controller.back());

    public final Trigger button;

    ControlScheme(Trigger button) {
        this.button = button;
    }
}
