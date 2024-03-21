// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.supersystems.PositionController;
import frc.robot.supersystems.TargetPosition;

public class Pivot extends PositionController {
  protected double AngleWeWantToSet=0;

  /** Creates a new Pivot_Subsystem. */

  public Pivot() {
    super(
      new TargetPosition(
        new TalonFX(Constants.Pivot.id, "CANivore"), Constants.Pivot.stowPosition, 
        Constants.Pivot.kP, Constants.Pivot.kV, Constants.Pivot.kG
      )
    );
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 100; // 80 rps cruise velocity
    // motionMagicConfigs.MotionMagicAcceleration = 310; // 160 rps/s acceleration (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    // periodic, run Motion Magic with slot 0 configs,
    // target position of 200 rotations
  }

  public Command setAngleCommand(DoubleSupplier angle){
    //System.out.println("Set angle " + angle.getAsDouble());
    return new Command () {
      @Override 
      public void initialize() {
        setPosition(angle);
      }

      @Override 
      public boolean isFinished() {
        return isAtAngle(angle);
      }
    };
  }

  public Command stowCommand() {
    //System.out.println("Stow command " + Constants.Pivot.stowPosition);
    return Commands.runOnce(() -> setPosition(Constants.Pivot.stowPosition));
  }

  /**
   * Returns to stowed position when interrupted (on trigger release).
   * */
  public Command autoAngleCommand(DoubleSupplier distance) {
    return new Command() {
      @Override
      public void execute() {
        if(distance.getAsDouble() < 1.1 || distance.getAsDouble() > 25 || getTargetAngle(distance) > 40)
          return;
          
        setPosition(() -> getTargetAngle(distance));
        AngleWeWantToSet = distance.getAsDouble();
      }

      @Override
      public void end(boolean interrupted) {
        setPosition(Constants.Pivot.stowPosition);
      }
    };
  }

  public boolean isAtAngle(DoubleSupplier targetAngle){
    return Math.abs(targetAngle.getAsDouble() - getPosition()) < 1.0;
  }

  public double getTargetAngle(DoubleSupplier distanceSupplier){
    double distance = distanceSupplier.getAsDouble();
    // Quintic return 39.48 + (-45.98 * distance) 
    //   + Math.pow(36.25 * distance, 2) 
    //   - Math.pow(11.62 * distance, 3)
    //   + Math.pow(1.707, 4)
    //   - Math.pow(-0.0095 * distance, 5);
    // Cosine squared return 2946 * Math.pow(Math.cos(0.01909 * distance - 0.08999), 2) - 2914;
    // Gaussian return -15.57 * -Math.pow(Math.pow((distance - 0.2858), 2) / Math.pow(-2.156, 2), Math.E) + 32.2;
    return -1.072 * Math.pow(distance, 2) + 10.10 * distance + 8.177;
    // Sine return 49.74 * Math.sin(0.2245 * distance + 0.5432) - 15.89;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target pivot angle", AngleWeWantToSet);
    SmartDashboard.putNumber("Actual pivot angle", getPosition());
    // This method will be called once per scheduler run
  }
}
