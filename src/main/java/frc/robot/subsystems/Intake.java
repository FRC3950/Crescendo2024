// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.misc.LimelightHelpers;
import frc.robot.supersystems.TargetVelocity;
import frc.robot.supersystems.VelocityController;
import frc.robot.constants.Constants;

public class Intake extends VelocityController {

  private final StatusSignal<ReverseLimitValue> beamBreak;

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
    
    beamBreak = getSensorSignal(Constants.Intake.indexerId);

    SmartDashboard.putBoolean("INTAKE", false);
  }

  public Command intakeForeverCommand() {
    return new Command() {
      @Override
      public void initialize() {
        applyInitialTargetVelocities();
      }

      @Override
      public void end(boolean interrupted){}

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
  

  public Command intakeCommand() {
    return new Command() {
      @Override
      public void initialize() {
        applyInitialTargetVelocities();
      }

      @Override
      public void end(boolean interrupted){
        stop();
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
        applyVelocities(() -> (-Constants.Intake.indexerActiveVelocity.getAsDouble()));
      }

      @Override 
      public void end(boolean interrupted) {
        stop();
      }
    };
  }

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

  public Command stopCommand() {
    return Commands.runOnce(super::stop);
  }

  public boolean noteIsIndexed() {
    return !beamBreak.getValue().equals(ReverseLimitValue.Open);
  }

  public void index() {
    
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
