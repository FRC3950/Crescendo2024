// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    /** Creates a new Elevator. */

    private final TalonFX left = new TalonFX(Constants.Elevator.leftId);
    private final TalonFX right = new TalonFX(Constants.Elevator.rightId);

    State state = State.STOWED;

    private static Elevator instance;

    private final double kV = 0.12;
    private final double kP = 0.01; //TODO

    private final double mCruiseVel = 10;
    private final double mAccel = 10;
    private final double mJerk = 100;

    private final MotionMagicVoltage mVoltage = new MotionMagicVoltage(0);
    private final TalonFXConfiguration talonFxConfig = new TalonFXConfiguration();
    private final Slot0Configs pidConfigs = talonFxConfig.Slot0;
    private enum State {
      TRAP (500),
      STOWED (0);

      final double encoder;

      State(double encoder){
          this.encoder = encoder;
      }
    }

    private Elevator() {
        pidConfigs.kV = kV;
        pidConfigs.kP = kP;

        var mConfigs = talonFxConfig.MotionMagic;

        mConfigs.MotionMagicCruiseVelocity = mCruiseVel;
        mConfigs.MotionMagicAcceleration = mAccel;
        mConfigs.MotionMagicJerk = mJerk;
        mVoltage.Slot = 0;

        left.getConfigurator().apply(pidConfigs);
        right.getConfigurator().apply(pidConfigs);

        right.setControl(new Follower(left.getDeviceID(), false));
    }

    public void setState(State newState) {
        state = newState;

        left.setControl(mVoltage.withPosition(newState.encoder));
    }

    public void stow() { // Most likely going to incorporate limit switch
        state = State.STOWED;

        left.setControl(mVoltage.withPosition(0));
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
