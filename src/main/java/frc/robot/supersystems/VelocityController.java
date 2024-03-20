// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.supersystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class VelocityController extends SubsystemBase {
  /** Creates a new VelocityController. */

  protected final TargetVelocity[] targets;

  protected VelocityController(TargetVelocity...targets) {
    this.targets = targets;
  }
  
  public void applyInitialTargetVelocities() {
    for(TargetVelocity target : targets)
      target.motor.setControl(target.velVoltage.withVelocity(target.targetVelocity));
  }

  public void applyVelocity(int motorId, double percent){
    for(TargetVelocity target : targets) {
      if(target.motor.getDeviceID() == motorId){
        target.motor.set(percent);
        return;
      }
    }
  }

  protected void applyVelocities(DoubleSupplier velocity) {
    for(TargetVelocity target : targets)
      target.motor.setControl(target.velVoltage.withVelocity(velocity.getAsDouble()));
  }

  public void stop() {
    for(TargetVelocity target : targets)
      target.motor.setControl(target.velVoltage.withVelocity(0));
  }

  protected double getVelocity(int motorId) {
    for(TargetVelocity target : targets){
      if(target.motor.getDeviceID() == motorId)
        return target.motor.getVelocity().getValueAsDouble();
    }

    return 0;
  }

  protected StatusSignal<ReverseLimitValue> getSensorSignal(int motorId) {
    for(TargetVelocity target : targets){
      if(target.motor.getDeviceID() == motorId)
        return target.motor.getReverseLimit();
    }

    return null;
  }

  @Override
  public void periodic() {}
}
