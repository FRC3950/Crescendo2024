// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import javax.sound.sampled.Control;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VortexTest extends SubsystemBase {
  /** Creates a new VortexTest. */

  private final CANSparkFlex motor1 = new CANSparkFlex(52, MotorType.kBrushless);
  private final CANSparkFlex motor2 = new CANSparkFlex(53, MotorType.kBrushless);

  private double motor1Velocity = 80;
  private double motor2Velocity = 80;

  private double kF = 0.0010987999;
  private double maxRpm = 430;

  private final CANSparkBase.ControlType controlType = ControlType.kVelocity;

  private final SparkPIDController pid1, pid2;

  public VortexTest() {
    SmartDashboard.putNumber("Motor 1 velocity", 0);
    SmartDashboard.putNumber("Motor 2 velocity", 0);

    SmartDashboard.putNumber("Motor 1 actual velocity", motor1.getEncoder().getVelocity());

    motor1.setIdleMode(IdleMode.kCoast);
    motor2.setIdleMode(IdleMode.kCoast);

    pid1 = motor1.getPIDController();
    pid2 = motor2.getPIDController();

    pid1.setFF(kF);
    pid1.setOutputRange(-1, 1);

    pid2.setFF(kF);
    pid2.setOutputRange(-1, 1);
  }

  public void engageMotor1(){
    motor1.set(motor1Velocity);
  }

  public void engageMotor2(){
    motor2.set(motor2Velocity);
  }

  public void intake(){
    pid1.setReference(motor1Velocity, controlType);
    pid2.setReference(motor2Velocity, controlType);
  }

  public void stopMotors() {
    pid1.setReference(0, controlType);
    pid2.setReference(0, controlType);
  }

  @Override
  public void periodic() {
    motor1Velocity = SmartDashboard.getNumber("Motor 1 velocity", 0);
    motor2Velocity = SmartDashboard.getNumber("Motor 2 velocity", 0);

    SmartDashboard.putNumber("Motor 1 actual velocity", motor1.getEncoder().getVelocity() * (motor1.getEncoder().getVelocityConversionFactor()));
  }
}
