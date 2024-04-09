// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.meta.EndType;
import lib.meta.EndsOn;
import lib.odometry.LimelightHelpers;
import lib.system.TargetVelocity;
import lib.system.VelocityController;

public class Intake extends VelocityController {

    public Intake() {
        super(
                new TargetVelocity( // Intake
                        new TalonFX(Constants.Intake.leftId),
                        new TalonFX(Constants.Intake.rightId),
                        65,
                        Constants.Intake.intakeKp,
                        Constants.Intake.intakeKv, false
                ),
                new TargetVelocity( // Indexer
                        new TalonFX(Constants.Intake.indexerId, "CANivore"),
                        40,
                        Constants.Intake.indexerKp,
                        Constants.Intake.indexerKv
                )
        );

        SmartDashboard.putBoolean("INTAKE", false);
    }

    private boolean noteIsIndexed() {
        return !targets[1].motor.getReverseLimit().getValue().equals(ReverseLimitValue.Open);
    }


    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT_OR_FINISH)
    public Command intakeCommand() {
        return new Command() {
            @Override
            public void initialize() {
                applyInitialTargetVelocities();
            }

            @Override
            public void end(boolean interrupted) {
                stop();
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
                applyVelocities(() -> (-Constants.Intake.indexerActiveVelocity.getAsDouble()));
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command ampOuttakeCommand() {
        return new Command() {
            @Override
            public void initialize() {
                applyVelocity(Constants.Intake.indexerId, () -> -Constants.Intake.indexerActiveVelocity.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT_OR_FINISH)
    public Command indexCommand() {
        return new Command() {
            @Override
            public void initialize() {
                applyVelocity(Constants.Intake.indexerId, Constants.Intake.indexerActiveVelocity);
            }

            @Override 
            public void end(boolean interrupted){
                stop();
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stopCommand() {
        return Commands.runOnce(super::stop);
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("INTAKE", noteIsIndexed());

        if (noteIsIndexed()) {
            LimelightHelpers.setLEDMode_ForceOn("limelight");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }
    }
}
