// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final TalonFX top = new TalonFX(Constants.Shooter.topId);
  private final TalonFX bottom = new TalonFX(Constants.Shooter.bottomId);

  State state = State.IDLE;

  private static Shooter instance;

  private final double kV = 0.12;
  private final double kP = 0.25; //TODO

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final Slot0Configs slot0Configs = new Slot0Configs();

  public static Shooter getInstance(){
      if(instance == null){
          instance = new Shooter();
      }
      return instance;
  }
  public enum State {
    OFF(0),
    IDLE(400),
    ACTIVE(4500);

    final double shooterVelocity;

    State(double shooterVelocity){this.shooterVelocity = shooterVelocity;}
  }

  private Shooter() {
      slot0Configs.kV = kV;
      slot0Configs.kP = kP;

      velocityVoltage.Slot = 0;

      top.getConfigurator().apply(slot0Configs);
      bottom.getConfigurator().apply(slot0Configs);

      bottom.setControl(new Follower(top.getDeviceID(), true));
  }

  public void setState( State newState){
    state = newState;

    top.setControl(velocityVoltage.withVelocity(newState.shooterVelocity));
  }

  public void idle(){
      state = State.IDLE;

      top.setControl(velocityVoltage.withVelocity(state.shooterVelocity));
  }

  public void stop() {
      state = State.OFF;

      top.setControl(velocityVoltage.withVelocity(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
