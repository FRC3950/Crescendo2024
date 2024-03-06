// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonFX leftMotor = new TalonFX(Constants.Intake.leftId);
  private final TalonFX rightMotor = new TalonFX(Constants.Intake.rightId);

  private final TalonFX indexer = new TalonFX(Constants.Intake.indexerId, "CANivore");

  private final LaserCan laserCan = new LaserCan(50);

  public State state = State.OFF;

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

  public Intake() {
    leftMotor.setInverted(true);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
  }

  public void stop() {
    state = State.OFF;

    leftMotor.set(0);
    indexer.set(0);
  }

  public void run(){
    state = State.INTAKE;
      leftMotor.set(0.70);
            indexer.set(0.30);

  }

  public void runIndexer(){
    indexer.set(0.5);
  }
  public void indexerOff(){
    indexer.set(0);
  }

  public void runIndexerBack(){
    indexer.set(-0.3 );
  }



  public void setState(State newState, boolean isForward){
    if(!noteIsIndexed() || !isForward){
      state = newState;

      leftMotor.set(newState.percentValue);
      indexer.set(newState.percentValue);
    }
 

    else {
      if (Shooter.getInstance().isActive()){

        state = State.INTAKE;

      }
      else{
        state = State.OFF;

      }
    }
  }
  
  public boolean noteIsIndexed() {
    return laserCan.getMeasurement().distance_mm < 25;
  }

  @Override
  public void periodic() {
    //System.out.println(laserCan.getMeasurement().distance_mm);
    // This method will be called once per scheduler run
  }
}
