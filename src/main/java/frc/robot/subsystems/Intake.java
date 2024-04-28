// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import lib.log.Loggable;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.meta.EndType;
import lib.meta.EndsOn;
import lib.odometry.LimelightHelpers;
import lib.state.VelocityState;
import lib.state.machines.VelocityStateMachine;

import java.util.HashSet;

public class Intake extends VelocityStateMachine implements Loggable {

    private static final VelocityState intakeActiveState = new VelocityState(
            Constants.Intake.intakeConfig, Constants.Intake.intakeActiveVelocity, 1
    );

    private static final VelocityState indexerActiveState = new VelocityState(
            Constants.Intake.indexerConfig, Constants.Intake.indexerActiveVelocity, 1
    );

    private static final HashSet<VelocityState> stopped = new HashSet<>() {{
        add(intakeActiveState.stopped());
        add(indexerActiveState.stopped());
    }};

    private static final HashSet<VelocityState> active = new HashSet<>() {{
        add(intakeActiveState);
        add(indexerActiveState);
    }};

    public Intake() {
        super(stopped);
    }

    public boolean noteIsIndexed() {
        var motor = indexerActiveState.talonConfig.motor;

        return motor.getReverseLimit().getValue().equals(ReverseLimitValue.Open);
    }

    @Override
    public void log() {
        SmartDashboard.putBoolean("INDEXED", noteIsIndexed());
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT_OR_FINISH)
    public Command intakeCommand() {
        return new Command() {
            @Override
            public void initialize() {
                acquireGoalState(active);
            }

            @Override
            public void end(boolean interrupted) {
                acquireGoalState(stopped);
            }

            @Override
            public boolean isFinished() {
                return noteIsIndexed();
            }
        };
    }


    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command outtakeCommand() {
        return new Command() {
            @Override
            public void initialize() {
                acquireNegativeGoalState(active);
            }

            @Override
            public void end(boolean interrupted) {
                acquireGoalState(stopped);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command ampOuttakeCommand() {
        return new Command() {
            @Override
            public void initialize() {
                acquireNegativeGoalState(indexerActiveState.asHashSet());
            }

            @Override
            public void end(boolean interrupted) {
                acquireGoalState(stopped);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command indexCommand() {
        return new Command() {
            @Override
            public void initialize() {
                acquireGoalState(indexerActiveState.asHashSet());
            }

            @Override
            public void end(boolean interrupted){
                acquireGoalState(stopped);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stopCommand() {
        return Commands.runOnce(() -> acquireGoalState(stopped));
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command intakeOnceCommand(){
        return Commands.runOnce(() -> {
            acquireGoalState(intakeActiveState.asHashSet());

            if(!noteIsIndexed()){
                acquireGoalState(active);
            }

            else{
                acquireGoalState(indexerActiveState.withChangedVelocity(() -> 0).asHashSet());
            }
        });
    }

    @Override
    public void periodic() {
        if (noteIsIndexed()) {
            LimelightHelpers.setLEDMode_ForceOn("limelight");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }
    }
}
