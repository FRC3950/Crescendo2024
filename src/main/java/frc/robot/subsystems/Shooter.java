// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Swerve;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.meta.EndType;
import lib.meta.EndsOn;
import lib.odometry.NoteKinematics;
import lib.system.control.TargetVelocity;
import lib.system.control.VelocityController;

public class Shooter extends VelocityController {

    public Shooter() {
        super(
                new TargetVelocity(
                        new TalonFX(Constants.Shooter.topId, "CANivore"),
                        new TalonFX(Constants.Shooter.bottomId, "CANivore"), Constants.Shooter.activeSpeed.getAsDouble(),
                        Constants.Shooter.kP, Constants.Shooter.kV, true
                )
        );
    }

    public boolean isLobbing = false;

    private double getVelocity() {
        if (targets.length < 1)
            return 0;

        return targets[0].motor.getVelocity().getValueAsDouble();
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stopCommand() {
        return Commands.runOnce(super::stop);
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command idleCommand() {
        return Commands.runOnce(() -> applyVelocities(Constants.Shooter.idleSpeed));
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command applyLobVelocity(DoubleSupplier distance) {
        return new Command() {
            @Override
            public void execute() {
                applyVelocity(targets[0].motor.getDeviceID(), () -> NoteKinematics.getLobVelocityRps(distance));
            }

            @Override
            public boolean isFinished() {
                return Math.abs(getVelocity() - NoteKinematics.getLobPivot(distance)) <= 1;
            }
        };
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command shootCommand(Intake intake, DoubleSupplier velocity, Swerve drive) {
        return new Command() {
            @Override
            public void initialize() {
                if(!isLobbing){
                    applyInitialTargetVelocities();
                }
            }

            @Override
            public void execute() {

                DoubleSupplier velocity = Constants.Shooter.activeSpeed;

                if(isLobbing){
                    velocity = () -> 1.75 * NoteKinematics.getLobVelocityRps(() -> NoteKinematics.getAllianceSpeakerDistance(drive));

                    if(velocity.getAsDouble() != 0){
                        applyVelocities(velocity);
                    }
                }

                if (getVelocity() >= velocity.getAsDouble() - 1) {
                    intake.applyVelocity(Constants.Intake.indexerId, Constants.Intake.indexerActiveVelocity);
                }
            }

            @Override
            public void end(boolean interrupted) {
                applyVelocities(Constants.Shooter.idleSpeed);
                intake.stop();
            }
        };
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.FINISH)
    public Command applyVelocitiesCommand() {
        return new Command() {
            @Override
            public void initialize() {
                applyInitialTargetVelocities();
            }

            @Override
            public boolean isFinished() {
                return Math.abs(getVelocity() - targets[0].targetVelocity) <= 1;
            }
        };
    }

    @Override
    public void periodic() {
    }
}
