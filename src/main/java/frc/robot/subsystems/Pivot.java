// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.NoteKinematics;
import frc.robot.supersystems.PositionController;
import frc.robot.supersystems.TargetPosition;

public class Pivot extends PositionController {

  protected double AngleWeWantToSet=0;

  /** Creates a new Pivot_Subsystem. */

  private final CANcoder cancoder = new CANcoder(Constants.Pivot.cancoderId, "CANivore");

  public Pivot() {
    super(
      new TargetPosition(
        new TalonFX(Constants.Pivot.id, "CANivore"), Constants.Pivot.stowPosition, 
        Constants.Pivot.kP, Constants.Pivot.kV, Constants.Pivot.kG
      )
    );
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 100; // 80 rps cruise velocity
    // motionMagicConfigs.MotionMagicAcceleration = 310; // 160 rps/s acceleration (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    // periodic, run Motion Magic with slot 0 configs,
    // target position of 200 rotations
  }

  private boolean isAtLimit() {
    return targetPosition.motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public Command setAngleInstantCommand(DoubleSupplier angle){
    return Commands.runOnce(() -> setPosition(angle));
  } 

  public Command setAngleCommand(DoubleSupplier angle){
    return new Command () {
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

  public Command stowCommand() {
    return Commands.runOnce(() -> setPosition(Constants.Pivot.stowPosition));
  }

  // Prevents limit switch snafus 
  public Command forceStowCommand() {
    return new Command() {
      @Override 
      public void initialize () {
        targetPosition.motor.setVoltage(-3.5);
      }

      @Override 
      public void end(boolean interrupted) {
        targetPosition.motor.setVoltage(0);
      }
    };
  }

  /**
   * Returns to stowed position when interrupted (on trigger release).
   * */
  public Command autoAngleCommand(DoubleSupplier distance) {
    return new Command() {
      @Override
      public void execute() {
        if(distance.getAsDouble() < 1.0 || distance.getAsDouble() > 25 || NoteKinematics.getTargetPivot(distance) > 0.25)
          return;
          
        setPosition(() -> NoteKinematics.getTargetPivot(distance));
        AngleWeWantToSet = distance.getAsDouble();
      }
    };
  }

  public Command NewAutoAngleCommand(DoubleSupplier distance) {
    return new Command() {
      @Override
      public void execute() {
        if(distance.getAsDouble() < 1.1 || distance.getAsDouble() > 25 || NoteKinematics.getTargetPivot(distance) > 0.25)
          return;
          
        setPosition(() -> NoteKinematics.getTargetPivot(distance));
        AngleWeWantToSet = distance.getAsDouble();
      }

      @Override
      public void end(boolean interrupted) {
        setPosition(Constants.Pivot.stowPosition);
      }
    };
  }

  public boolean isAtAngle(DoubleSupplier targetAngle){
    return Math.abs(targetAngle.getAsDouble() - getPosition()) < 1.0;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Target pivot angle", AngleWeWantToSet);
    // SmartDashboard.putNumber("Actual pivot angle", getPosition());

    if(isAtLimit()){
     // targetPosition.motor.setPosition(0.003 - 0.25);
      cancoder.setPosition(0.003);
    }
  }
}
