// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.LimelightHelpers;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonFX leftMotor = new TalonFX(Constants.Intake.leftId);
  private final TalonFX rightMotor = new TalonFX(Constants.Intake.rightId);

  private final TalonFX indexer = new TalonFX(Constants.Intake.indexerId, "CANivore");

  private final StatusSignal<ReverseLimitValue> beamBreak = indexer.getReverseLimit();

  public State state = State.OFF;

  private final VelocityVoltage indexerVelocity = new VelocityVoltage(0);
  private final Slot0Configs indexerConfigs = new Slot0Configs();

  private final double kP = 0.01;
  private final double kV = 0.12;

  public final IntakeCommands commands = new IntakeCommands();

  public enum State {

    INTAKE(0.7, 38),
    OUTTAKE(-0.7, -38),
    INDEX(0, 38),
    OFF(0, 0);

    final double intakePercent, indexerVelocity;

    State(double intakePercent, double indexerVelocity){
        this.intakePercent = intakePercent;
        this.indexerVelocity = indexerVelocity;
    }
  }

  public Intake() {

    indexerConfigs.kP = kP;
    indexerConfigs.kV = kV;

    indexerVelocity.Slot = 0;

    leftMotor.setInverted(true);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));

    SmartDashboard.putBoolean("INTAKE", false);
  }

  public final class IntakeCommands {
    public Command intakeCommand() {
        return new Command() {
            @Override
            public void initialize() {
                setState(State.INTAKE);
            }

            @Override
            public void end(boolean interrupted) {
                setState(State.OFF);
            }

            @Override
            public boolean isFinished() {
                return noteIsIndexed();
            }
        };
    }

    public Command outtakeCommand() {
        return new Command() {
            @Override
            public void initialize() {
                setState(State.OUTTAKE);
            }

            @Override
            public void end(boolean interrupted){
                setState(State.OFF);
            }
        };
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> setState(State.OFF));
    }
  }

  public void setState(State newState){
    if(!noteIsIndexed() || newState == State.OUTTAKE || newState == State.INDEX) {
        state = newState;

        leftMotor.set(newState.intakePercent);
        indexer.setControl(indexerVelocity.withVelocity(newState.indexerVelocity));
    }

    else {
      state = State.OFF;
    }
  }

  public boolean noteIsIndexed() {
    return !beamBreak.getValue().equals(ReverseLimitValue.Open);
  }

  @Override
  public void periodic() {
    beamBreak.refresh();

    SmartDashboard.putBoolean("INTAKE", noteIsIndexed());

    if(noteIsIndexed()){
      LimelightHelpers.setLEDMode_ForceOn("limelight");
    }
    else {
      LimelightHelpers.setLEDMode_ForceOff("limelight");
    }
  }
}
