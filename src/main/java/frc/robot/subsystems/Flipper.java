// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.supersystems.PositionController;
import frc.robot.supersystems.TargetPosition;

public class Flipper extends PositionController {

  public Flipper() {
    super(
      new TargetPosition(
        new TalonFX(Constants.Flipper.id, "CANivore"), Constants.Flipper.stowPosition,
        Constants.Flipper.kP, Constants.Flipper.kV, Constants.Flipper.kG
      )
    );
  }
  
  public Command stowCommand() {
    return Commands.runOnce(() -> setPosition(Constants.Flipper.stowPosition));
  } 

  public Command ampCommand() {
    System.out.println("AMP");
    return Commands.runOnce(() -> setPosition(Constants.Flipper.ampPosition));
  }

  @Override
  public void periodic() {}
}
