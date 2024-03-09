// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Instant;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
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

  public enum State {

    INTAKE(0.7),
    OUTTAKE(-0.7),
    OFF(0);

    final double percentValue;

    State(double percentValue){this.percentValue = percentValue;}
  }

  private static Intake instance;

  public static Intake getInstance(){
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  private Intake() {

    indexerConfigs.kP = kP;
    indexerConfigs.kV = kV;

    indexerVelocity.Slot = 0;

    leftMotor.setInverted(true);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));

    SmartDashboard.putBoolean("INTAKE", false);
  }

  public void stop() {
    state = State.OFF;

    leftMotor.set(0);
    indexer.set(0);
  }

  public void run(){
    state = State.INTAKE;

    leftMotor.set(0.7);
    indexer.set(0.30);
  }


  public void autoIntake() {
    //state = State.INTAKE;
    if(!noteIsIndexed()){
      leftMotor.set(0.80);
      indexer.set(0.35);
    }

    else {
      leftMotor.set(0);
      indexer.set(0);
    }
  }

  public void runIndexer(){
    indexer.setControl(indexerVelocity.withVelocity(38));
  }

  public void runIndexerIntake() {
    indexer.setControl(indexerVelocity.withVelocity(34));
  }

  public void indexerOff(){
    indexer.set(0);
  }

  public void runIndexerBack(){
    indexer.set(-0.3);
  }

  public Command intakeUntilNoteCommand(){
   return Commands.startEnd(this::run, this::stop, this)
   .until(() -> noteIsIndexed())
   .andThen(this::runIndexerBack).until(() -> !noteIsIndexed());
  }

  public Command autoIntakeCommand() {
    return Commands.startEnd(
      this::run, this::stop, this).until(()->noteIsIndexed());
  }

  public void setState(State newState, boolean isForward){
    if(!noteIsIndexed() || !isForward){
      state = newState;

      leftMotor.set(newState.percentValue);
      indexer.set(newState.percentValue);
    }
 

    else {
      if (Shooter.getInstance().isActive()) {
        state = State.INTAKE;
      }
      else {
        state = State.OFF;
      }
    }
  }
  
  public boolean noteIsIndexed() {
    //System.out.println(laserCan.getMeasurement().distance_mm < 150);
  
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
    //System.out.println(laserCan.getMeasurement().distance_mm);
    // This method will be called once per scheduler run
  }
}
