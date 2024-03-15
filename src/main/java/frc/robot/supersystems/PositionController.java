// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.supersystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class PositionController extends SubsystemBase {
  /** Creates a new StateController. */
  private final TargetPosition targetPosition;
  private final MotionMagicVoltage mmVoltage;

  protected PositionController(TargetPosition targetPosition) {

    this.targetPosition = targetPosition;
    mmVoltage = targetPosition.mmVoltage;

    setPosition(targetPosition.initalPosition);
  }

  protected void setPosition(DoubleSupplier position){
    targetPosition.motor.setControl(mmVoltage.withPosition(position.getAsDouble()));
  }

  protected void setZero(){
    targetPosition.motor.setControl(mmVoltage.withPosition(0));
  }

  protected double getPosition() {
    return targetPosition.motor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {}
}
