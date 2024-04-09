// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.meta.EndType;
import lib.meta.EndsOn;
import lib.odometry.NoteKinematics;
import lib.system.PositionController;
import lib.system.TargetPosition;

import java.util.function.DoubleSupplier;

public class Pivot extends PositionController {

    private final CANcoder cancoder = new CANcoder(Constants.Pivot.cancoderId, "CANivore");

    private final Slot1Configs slot1Pid = new Slot1Configs();

    public Pivot() {
        super(
                new TargetPosition(
                        new TalonFX(Constants.Pivot.id, "CANivore"), Constants.Pivot.stowPosition,
                        Constants.Pivot.kP, Constants.Pivot.kV, Constants.Pivot.kG, true
                )
        );

        slot1Pid.kP = 0.05;
    }

    private boolean isAtAngle(DoubleSupplier targetAngle) {
        return Math.abs(targetAngle.getAsDouble() - getPosition()) < 1.0;
    }

    private boolean isAtLimit() {
        return targetPosition.motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command setAngleInstantCommand(DoubleSupplier angle) {
        return Commands.runOnce(() -> setPosition(angle));
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.FINISH)
    public Command setAngleCommand(DoubleSupplier angle) {
        return new Command() {
            @Override
            public void initialize() {
                setPosition(angle);
            }

            @Override
            public boolean isFinished() {
                return isAtAngle(angle);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stowCommand() {
        return Commands.runOnce(() -> setPosition(Constants.Pivot.stowPosition));
    }

    @CommandBehavior(behavior = CommandType.INITIALIZE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command forceStowCommand() {
        return new Command() {
            @Override
            public void initialize() {
                targetPosition.motor.setVoltage(-3.5);
            }

            @Override
            public void end(boolean interrupted) {
                targetPosition.motor.setVoltage(0);
            }
        };
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command autoAngleCommand(DoubleSupplier distance) {
        return new Command() {
            @Override
            public void execute() {
                if (distance.getAsDouble() < 1.0 || distance.getAsDouble() > 25 || NoteKinematics.getTargetPivot(distance) > 0.25)
                    return;

                setPosition(() -> NoteKinematics.getTargetPivot(distance));
            }
        };
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command lobAngleCommand(DoubleSupplier distance){
        return new Command() {
            @Override
            public void execute() {
                if(distance.getAsDouble() < 1.0 || distance.getAsDouble() > 25 || NoteKinematics.getLobPivot(distance) > 0.25 || NoteKinematics.getLobPivot(distance) < 0) {
                    System.out.println(NoteKinematics.getLobPivot(distance));
                    return;
                }
                
                setPosition(() -> NoteKinematics.getLobPivot(distance));
            }
        };
    }

    @Override
    public void periodic() {
        if (isAtLimit()) {
            cancoder.setPosition(0);
        }
    }
}
