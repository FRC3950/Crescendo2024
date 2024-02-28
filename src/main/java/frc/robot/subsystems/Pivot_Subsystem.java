// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot_Subsystem extends SubsystemBase {
  /** Creates a new Pivot_Subsystem. */

  private final TalonFX pivotMotor = new TalonFX(Constants.PivotMotor);
  private final double gearRatio = 128.0;
  private final double RotationPerAngle = gearRatio / 360.0;
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  private double distanceFromTarget;
  private double kDegreesPerDistanceAwayFromTarget = 7; // 2 degrees per meter

  public Pivot_Subsystem() {


var talonFXConfigs = new TalonFXConfiguration();

// set slot 0 gains
var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = 0.12; // add 0.24 V to overcome friction
slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
// PID runs on position
slot0Configs.kP = 0.8; //og 4.8
slot0Configs.kI = 0;
slot0Configs.kD = 0.02;

// set Motion Magic settings
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
motionMagicConfigs.MotionMagicAcceleration = 120; // 160 rps/s acceleration (0.5 seconds)
motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

pivotMotor.getConfigurator().apply(talonFXConfigs, 0.050);

// periodic, run Motion Magic with slot 0 configs,
// target position of 200 rotations
m_motmag.Slot = 0;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle", pivotMotor.getPosition().getValue());
    // This method will be called once per scheduler run
  }

  public void adjustAngle(double angle){
    SmartDashboard.putNumber("angle diff from intial", angle);
    SmartDashboard.putNumber("AngleOffVertical", 20 + angle);
    pivotMotor.setControl(m_motmag.withPosition(angle * RotationPerAngle));

  }

  public Command autoAngleCommand(DoubleSupplier distanceFromTarget){
   


    return new Command(){
      @Override
      public void initialize() {
        // TODO Auto-generated method stub
      }

      @Override
      public void execute() {
            var angleToPivotDownToFromInitial = distanceFromTarget.getAsDouble() * kDegreesPerDistanceAwayFromTarget;
            SmartDashboard.putNumber("distance", distanceFromTarget.getAsDouble());

        adjustAngle(angleToPivotDownToFromInitial);
      }

      @Override
      public void end(boolean interrupted) {
        // TODO Auto-generated method stub
      }

      @Override
      public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
      }
    };
  }
}
