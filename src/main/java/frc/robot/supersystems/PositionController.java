// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.supersystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class PositionController extends SubsystemBase {
  /** Creates a new StateController. */
  private final TalonFX controlledMotor;
  private final MotionMagicVoltage mmVoltageController;

  protected PositionController(int canId, String busName, DoubleSupplier initialPosition) {
    controlledMotor = new TalonFX(canId, busName);
    mmVoltageController = new MotionMagicVoltage(initialPosition.getAsDouble());

    setPosition(initialPosition);
  }

  protected void setPosition(DoubleSupplier position){
    controlledMotor.setControl(mmVoltageController.withPosition(position.getAsDouble()));
  }

  protected void setZero(){
    controlledMotor.setControl(mmVoltageController.withPosition(0));
  }

  protected double getPosition() {
    return controlledMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
