package frc.robot.xbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public enum ControlScheme {
// Manipulator
//    SHOOT_SPEAKER(Controller.MANIPULATOR.leftBumper()),
//    FORCE_STOW(Controller.MANIPULATOR.rightBumper()),
//    SHOOT(Controller.MANIPULATOR.rightTrigger(0.5)),
//    SHOOT_LOB(Controller.MANIPULATOR.pov(0)),
//    SCORE_AMP(Controller.MANIPULATOR.x()),
//    SCORE_TRAP(Controller.MANIPULATOR.pov(180)),
//
//    AIM_AUTO(Controller.MANIPULATOR.leftTrigger(0.5)),
//
//    INTAKE(Controller.MANIPULATOR.a()),
//    OUTTAKE(Controller.MANIPULATOR.b()),
//
//    AMP_OUTTAKE(Controller.MANIPULATOR.y()),

    // Single-operator commands
    SHOOT_SPEAKER(Controller.DRIVER.leftBumper()),
    FORCE_STOW(Controller.DRIVER.rightBumper()),
    SHOOT(Controller.DRIVER.rightTrigger(0.5)),
    SHOOT_LOB(Controller.DRIVER.pov(0)),
    SCORE_AMP(Controller.DRIVER.x()),

    AIM_AUTO(Controller.DRIVER.leftTrigger(0.5)),

    INTAKE(Controller.DRIVER.a()),
    OUTTAKE(Controller.DRIVER.b()),

    RESET_HEADING(Controller.DRIVER.y()),
    PATH_AMP(Controller.DRIVER.start()),
    PATH_SPEAKER(Controller.DRIVER.back());
    // PATH_STAGE(Controller.DRIVER.back());

    public final Trigger button;

    ControlScheme(Trigger button) {
        this.button = button;
    }
}
