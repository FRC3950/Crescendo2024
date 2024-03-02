// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonFX leftMotor = new TalonFX(Constants.Intake.leftId);
  private final TalonFX rightMotor = new TalonFX(Constants.Intake.rightId);

  private final TalonFX indexer = new TalonFX(Constants.Intake.indexerId);

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
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
  }

  public void stop() {
    state = State.OFF;

    leftMotor.set(0);
    indexer.set(0);
  }

  public void setState(State newState){
    if(!noteIsIndexed()){
      state = newState;
      leftMotor.set(newState.percentValue);
    }

    else {
      state = State.OFF;
    }
  }
  
  public boolean noteIsIndexed() {
    return laserCan.getMeasurement().distance_mm < 10;
  }

  @Override
  public void periodic() {
    //System.out.println(laserCan.getMeasurement().distance_mm);
    // This method will be called once per scheduler run
  }
}
