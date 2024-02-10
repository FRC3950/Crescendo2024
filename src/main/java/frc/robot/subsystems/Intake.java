// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonFX frontMotor = new TalonFX(Constants.Intake.frontId);
  private final TalonFX backMotor = new TalonFX(Constants.Intake.backId);

  public enum IntakeState {
    
    FORWARD(0.7),
    REVERSE(-0.8),
    OFF(0);
    
    private final double motorPercent;

    private IntakeState(double motorPercent){
      this.motorPercent = motorPercent;
    }
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

  public void intake(double percent) {
    frontMotor.set(percent);
  }

  public void stop(){
    frontMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
