// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import lib.log.Loggable;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.meta.EndType;
import lib.meta.EndsOn;
import lib.odometry.ScoringKinematics;
import lib.state.VelocityState;
import lib.state.machines.VelocityStateMachine;

public class Shooter extends VelocityStateMachine implements Loggable {

    public static final VelocityState activeState = new VelocityState(Constants.Shooter.config, Constants.Shooter.activeSpeed, 0.5);
    public static final VelocityState lobState = activeState.withChangedVelocity(() -> 20);

    public static final HashSet<VelocityState> idle = activeState.withChangedVelocity(Constants.Shooter.idleSpeed).asHashSet();
    public static final HashSet<VelocityState> stopped = activeState.stopped().asHashSet();

    private static final HashSet<VelocityState> active = activeState.asHashSet();
    private static final HashSet<VelocityState> lob = lobState.asHashSet();


    public Shooter() {
        super(stopped);
    }

    @Override
    public void log() {}

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stopCommand() {
        return Commands.runOnce(() -> acquireGoalState(stopped));
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command idleCommand() {
        return Commands.runOnce(() -> acquireGoalState(idle));
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command applyLobState(DoubleSupplier distance) {
        return new Command() {
            @Override
            public void initialize() {
                acquireGoalState(lob);
            }

            @Override
            public void execute() {
                updateGoalState(() -> 1.75 * ScoringKinematics.getLobVelocityRps(distance));
            }

            @Override
            public boolean isFinished() {
                return isAtState(goalState);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.FINISH)
    public Command applyShootState() {
        return new Command() {
            @Override
            public void initialize() {
                acquireGoalState(active);
            }

            @Override
            public boolean isFinished() {
                return isAtState(active);
            }
        };
    }
}
