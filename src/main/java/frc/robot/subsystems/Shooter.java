// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  PositionState currentState = PositionState.STOWED;

  // Shooter wheel velocity, pivot velocity  
  private enum PositionState {
    
    STOWED(200),
    PRIMED(1500),
    STAGE_SCORE(300);

    private final double shooterVelocity;

    private PositionState(int shooterVelocity){
      this.shooterVelocity = shooterVelocity;
    }
  }

  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
