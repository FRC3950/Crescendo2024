// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import lib.log.Loggable;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.meta.EndType;
import lib.meta.EndsOn;
import lib.state.PositionState;
import lib.state.machines.PositionStateMachine;


public class Flipper extends PositionStateMachine implements Loggable {

    private static final PositionState amped = new PositionState(
            Constants.Flipper.config, () -> -1.0, 0.01
    );

    private static final PositionState stowed = amped.withChangedPosition(() -> -0.25);

    public Flipper() {
        super(stowed);
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Flipper pos", amped.talonConfig.motor.getPosition().getValueAsDouble());
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stowCommand() {
        return Commands.runOnce(() -> acquireGoalState(stowed));
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command ampCommand(Pivot pivot) {
        return new Command() {

            @Override
            public void execute() {
                if(pivot.getPosition() > 0.15){
                    acquireGoalState(amped);
                }
            }

            @Override
            public void end(boolean interrupted){
                acquireGoalState(stowed);
            }
        };
    }
}
