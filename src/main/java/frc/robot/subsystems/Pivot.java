// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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

import java.util.function.DoubleSupplier;

public class Pivot extends PositionStateMachine implements Loggable {

    private final CANcoder cancoder = new CANcoder(Constants.Pivot.cancoderId, "CANivore");

    private static final PositionState amped = new PositionState(
            Constants.Pivot.config, Constants.Pivot.ampPosition, 0.05
    );

    private static final PositionState stowed = amped.withChangedPosition(Constants.Pivot.stowPosition);
    private static final PositionState active = amped.withChangedPosition(Constants.Pivot.speakerPosition);

    private final TalonFX motor = amped.talonConfig.motor;

    public Pivot() {
        super(stowed);
    }

    @Override
    public void log() {}

    private boolean isAtLimit() {
        return currentState.talonConfig.isAtReverseLimit();
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stowInstantCommand() {
        return Commands.runOnce(() -> acquireGoalState(stowed));
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command setAngleInstantCommand(DoubleSupplier angle) {
        return Commands.runOnce(() -> {
            active.updateGoal(angle);
            acquireGoalState(active);
        });
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command ampCommand() {
        return Commands.runOnce(() -> acquireGoalState(amped));
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command forceStowCommand() {
        return new Command() {
            @Override
            public void initialize() {
                motor.setVoltage(-6);
            }

            @Override
            public void end(boolean interrupted) {
                motor.setVoltage(0);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.FINISH)
    public Command setAngleCommand(DoubleSupplier angle) {
        return new Command() {
            @Override
            public void initialize() {
                active.updateGoal(angle);
                acquireGoalState(active);
            }

            @Override
            public boolean isFinished() {
                return isAtState(active);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command continuousAngleCommand(DoubleSupplier angle) {
        return new Command() {
            @Override
            public void execute() {
                if (angle.getAsDouble() < -0.01 || angle.getAsDouble() > 0.35)
                    return;

                active.updateGoal(angle);
                acquireGoalState(active);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT_OR_FINISH)
    public Command stowCommand(){
        return new Command() {
            @Override
            public void initialize() {
                acquireGoalState(stowed);
            }

            @Override
            public boolean isFinished(){
                return isAtLimit();
            }
        };
    }

    @Override
    public void periodic() {
        if (isAtLimit()) {
            cancoder.setPosition(0);
            currentState = stowed;
        }
    }
}
