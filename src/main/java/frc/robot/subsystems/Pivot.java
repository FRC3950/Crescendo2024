// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot_Subsystem. */

  private final TalonFX pivotMotor = new TalonFX(Constants.pivotId,"CANivore");

  private final double gearRatio = 128.0;
  private final double RotationPerAngle = gearRatio / 360.0;

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  private double distanceFromTarget;
  private double kDegreesPerDistanceAwayFromTarget = 7; // 2 degrees per meter

  private double distance =1.35;

  private double angle =0;

  

  public Pivot() {
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    //slot0Configs.kS = 0.12; // add 0.24 V to overcome friction
    //slot0Configs.kV = 0.1175; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
 //   slot0Configs.kP = 1.85; //og 4.8
  //  slot0Configs.kI = 0;
  //  slot0Configs.kG = 0.4;
    //slot0Configs.kD = 0.02;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 90; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 300; // 160 rps/s acceleration (0.5 seconds)
   // motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    //pivotMotor.getConfigurator().apply(talonFXConfigs);

    // periodic, run Motion Magic with slot 0 configs,
    // target position of 200 rotations
    m_motmag.Slot = 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot angle", pivotMotor.getPosition().getValue());
    // This method will be called once per scheduler run
  }

  public void adjustAngle(double angle){
   // //SmartDashboard.putNumber("angle diff from intial", angle);
  // // SmartDashboard.putNumber("AngleOffVertical", 20 + angle);
    if (angle < 55) {
      pivotMotor.setControl(m_motmag.withPosition(angle));
    }
  }

  public boolean isAtAngle(double targetAngle){
    return Math.abs(targetAngle - pivotMotor.getPosition().getValueAsDouble())<0.5;
  }

  public double currentAngle(){
    return pivotMotor.getPosition().getValue() / RotationPerAngle;
  }

  public boolean isAtTargetAngle(double targetAngle){
    return Math.abs(currentAngle() - targetAngle) < 1;
  }

  public void setOurDistance(DoubleSupplier ourDistanceFromShot){
    distance = ourDistanceFromShot.getAsDouble();
    angle = -1.094 * Math.pow(distance, 2) + 9.854 * distance + 6.827;
    //angle = -1.808 * Math.pow(distance, 2) + 14.7 * distance + 1.332;
    //System.out.println(angle);
  }

  public void setDutyCycle(){

    pivotMotor.set(-0.15);
  }


  public Command autoAngleNewCommand(DoubleSupplier ourDistanceFromShot){
    return Commands.runEnd(      
      () -> {
        if(angle < -4 || angle > 45) { 
          return;
        }
        adjustAngle(angle);
        setOurDistance(ourDistanceFromShot);
      }, 
      () -> adjustAngle(-4), this);
  }
  

  public Command autoAngleCommand(DoubleSupplier distanceFromTarget) {
    return new Command(){
      @Override
      public void initialize() {
        // TODO Auto-generated method stub
      }

      @Override
      public void execute() {
        var angleToPivotDownToFromInitial = distanceFromTarget.getAsDouble() * kDegreesPerDistanceAwayFromTarget;
            //SmartDashboard.putNumber("distance", distanceFromTarget.getAsDouble());

        adjustAngle(angleToPivotDownToFromInitial);
      }

      @Override
      public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        adjustAngle(0);
      }

      @Override
      public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
      }
    };
  }
}
