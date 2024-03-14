// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.supersystems.PositionController;

public class Flipper extends PositionController {
  
  public Command stowCommand() {
    return Commands.runOnce(() -> setPosition(Constants.Flipper.stowPosition));
  } 

  public Command ampCommand() {
    return Commands.runOnce(() -> setPosition(Constants.Flipper.ampPosition));
  }

  public Flipper() {
    super(Constants.Flipper.id, "CANivore", Constants.Flipper.stowPosition);
  }

  @Override
  public void periodic() {}
}
