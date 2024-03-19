package frc.robot.xbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public enum ControlScheme {
    // Manipulator
    SHOOT_SPEAKER(Controller.MANIPULATOR.controller.leftBumper()),
    SHOOT_STAGE(Controller.MANIPULATOR.controller.rightBumper()),
    SHOOT(Controller.MANIPULATOR.controller.rightTrigger(0.5)),
    SCORE_AMP(Controller.MANIPULATOR.controller.x()),

    AIM_AUTO(Controller.MANIPULATOR.controller.leftTrigger(0.5)),

    INTAKE(Controller.MANIPULATOR.controller.a()),
    OUTTAKE(Controller.MANIPULATOR.controller.b()),

    TEST_UTIL(Controller.MANIPULATOR.controller.y()),

    // Driver
    RESET_HEADING(Controller.DRIVER.controller.y()),
    PATH_SPEAKER(Controller.DRIVER.controller.leftBumper());

    public final Trigger button;

    ControlScheme(Trigger button){
        this.button = button;
    }
}
