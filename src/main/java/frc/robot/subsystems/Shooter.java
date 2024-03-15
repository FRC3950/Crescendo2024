// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;
import frc.robot.supersystems.TargetVelocity;
import frc.robot.supersystems.VelocityController;

public class Shooter extends VelocityController {
  /** Creates a new Shooter. */

  public Shooter() {
    super(
      new TargetVelocity(
        new TalonFX(Constants.Shooter.topId, "CANivore"), 
        new TalonFX(Constants.Shooter.bottomId, "CANivore"), Constants.Shooter.activeSpeed.getAsDouble(), 
        Constants.Shooter.kP, Constants.Shooter.kV, true
      )
    );
  }

  public Command stopCommand() {
    return Commands.runOnce(super::stop);
  }

  public Command idleCommand() {
    return Commands.runOnce(() -> applyVelocities(Constants.Shooter.idleSpeed));
  }

  public Command shootCommand(Intake intake) {
    return new Command() {
      @Override 
      public void initialize() {
        applyInitialTargetVelocities();
      }

      @Override 
      public void execute() {
        if(getVelocity(Constants.Shooter.topId) >= Constants.Shooter.activeSpeed.getAsDouble()){
          intake.applyInitialTargetVelocities();
        }
      }

      @Override 
      public void end(boolean interrupted){
        applyVelocities(Constants.Shooter.idleSpeed);
        intake.stop();
      }
    };
  }

  // Used in aim-shoot commands 
  public Command applyVelocitiesCommand() {
    return new Command() {
      @Override
      public void initialize() {
        applyInitialTargetVelocities();
      }

      @Override 
      public boolean isFinished(){
        return Math.abs(getVelocity(Constants.Shooter.topId)) >= targets[0].targetVelocity - 1;
      }
    };
  }

  @Override
  public void periodic(){}
}
