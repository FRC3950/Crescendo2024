// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonFX frontMotor = new TalonFX(Constants.Intake.frontId);
  private final TalonFX backMotor = new TalonFX(Constants.Intake.backId);

  State state = State.OFF;

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
      backMotor.setControl(new Follower(frontMotor.getDeviceID(), false));
  }

  public void stop() {
      state = State.OFF;

      frontMotor.set(0);
  }

  public void setState(State newState){
      state = newState;

      frontMotor.set(newState.percentValue);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
