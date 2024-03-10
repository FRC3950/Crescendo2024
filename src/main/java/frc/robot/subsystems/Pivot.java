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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot_Subsystem. */

    private final TalonFX pivotMotor = new TalonFX(Constants.pivotId,"CANivore");

    private final double gearRatio = 128.0;
    private final double RotationPerAngle = gearRatio / 360.0;

    final MotionMagicVoltage pivotMotMag = new MotionMagicVoltage(0);

    private final double kDegreesPerDistanceAwayFromTarget = 7; // 2 degrees per meter

    public final PivotCommands commands = new PivotCommands();

  public Pivot() {
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    // TODO do this programmatically
    // slot0Configs.kS = 0.12;
    // slot0Configs.kV = 0.1175;
    // slot0Configs.kP = 1.85; //og 4.8
    // slot0Configs.kI = 0;
    // slot0Configs.kG = 0.4;
    // slot0Configs.kD = 0.02;
    // set Motion Magic settings

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 90; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 300; // 160 rps/s acceleration (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    // periodic, run Motion Magic with slot 0 configs,
    // target position of 200 rotations
    pivotMotMag.Slot = 0;
  }

  public class PivotCommands {
      public Command setAngleCommand(DoubleSupplier angle) {
          return Commands.runOnce(() -> Pivot.this.setAngle(angle), Pivot.this);
      }

      public Command stowCommand() {
          // Could do this with voltage control if kG were initialized here (instead of phoenix tuner)
          return Commands.runOnce(() -> setAngleCommand(() -> -4), Pivot.this);
      }

      /**
       * Returns to stowed position when interrupted (on trigger release).
       * */
      public Command autoAngleCommand(DoubleSupplier distance) {
          return new Command() {
              @Override
              public void execute() {
                  if(distance.getAsDouble() < -4 || distance.getAsDouble() > 55)
                      return;
                  Pivot.this.setAngle(() -> getTargetAngle(distance));
              }

              @Override
              public void end(boolean interrupted) {
                 Pivot.this.setAngle(() -> -4);
              }
          };
      }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot angle", pivotMotor.getPosition().getValue());
    // This method will be called once per scheduler run
  }

  public void setAngle(DoubleSupplier angle){
   // //SmartDashboard.putNumber("angle diff from intial", angle);
  // // SmartDashboard.putNumber("AngleOffVertical", 20 + angle);
    if (angle.getAsDouble() < 55) {
      pivotMotor.setControl(pivotMotMag.withPosition(angle.getAsDouble()));
    }
  }

  public boolean isAtAngle(DoubleSupplier targetAngle){
    return Math.abs(targetAngle.getAsDouble() - pivotMotor.getPosition().getValueAsDouble())<0.5;
  }

  public double getTargetAngle(DoubleSupplier distanceSupplier){
      double distance = distanceSupplier.getAsDouble();
      return -1.094 * Math.pow(distance, 2) + 9.854 * distance + 6.827;
  }
}
