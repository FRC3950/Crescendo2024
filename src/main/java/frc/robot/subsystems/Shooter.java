// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final TalonFX top = new TalonFX(Constants.Shooter.topId, "CANivore");
  private final TalonFX bottom = new TalonFX(Constants.Shooter.bottomId, "CANivore");

  State state = State.IDLE;

  private final double kV = 0.112;
  private final double kP = 0.005; //TODO

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final Slot0Configs slot0Configs = new Slot0Configs();

  public ShooterCommands commands = new ShooterCommands();
  public enum State {
    OFF(0),
    IDLE(10),
    ACTIVE(70);

    double shooterRps;

    State(double shooterRps){this.shooterRps = shooterRps;}
  }

  public Shooter() {
      slot0Configs.kV = kV;
      slot0Configs.kP = kP;

      velocityVoltage.Slot = 0;

      top.getConfigurator().apply(slot0Configs);
      bottom.getConfigurator().apply(slot0Configs);

      bottom.setControl(new Follower(top.getDeviceID(), true));
  }

  public class ShooterCommands {
      public Command shoot(Intake intake) {
          return new Command() {
              @Override
              public void execute() {
                  setState(State.ACTIVE);
                  if (getRpm() > 66)
                      intake.setState(Intake.State.INDEX);
              }

              @Override
              public void end(boolean interrupted) {
                  setState(State.IDLE);
                  intake.setState(Intake.State.OFF);
              }
          };
      }
  }

  public double getRpm() {
    return top.getVelocity().getValue();
  }

  public void setState(State newState){
    state = newState;

    top.setControl(velocityVoltage.withVelocity(newState.shooterRps));
  }

}
