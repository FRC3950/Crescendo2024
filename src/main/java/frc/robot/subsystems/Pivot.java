// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.supersystems.PositionController;

public class Pivot extends PositionController {
  /** Creates a new Pivot_Subsystem. */

  public Pivot() {
    super(Constants.Pivot.id, "CANivore", () -> 0);
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 100; // 80 rps cruise velocity
    // motionMagicConfigs.MotionMagicAcceleration = 310; // 160 rps/s acceleration (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    // periodic, run Motion Magic with slot 0 configs,
    // target position of 200 rotations
  }

  public Command setAngleCommand(DoubleSupplier angle){
    // Stows pivot if it is unsafe
    return Commands.runOnce(() -> setPosition(angle.getAsDouble() >= 65 ? () -> -4 : angle));
  }

  public Command stowCommand() {
    return Commands.runOnce(() -> setPosition(() -> -4));
  }

  /**
       * Returns to stowed position when interrupted (on trigger release).
       * */
  public Command autoAngleCommand(DoubleSupplier distance) {
    return new Command() {
        @Override
        public void execute() {
            if(distance.getAsDouble() < -4 || distance.getAsDouble() > 25)
                return;
            setPosition(() -> getTargetAngle(distance));
        }

        @Override
        public void end(boolean interrupted) {
            setPosition(() -> -4);
        }
    };
  }

  public boolean isAtAngle(DoubleSupplier targetAngle){
    return Math.abs(targetAngle.getAsDouble() - getPosition())<0.5;
  }

  public double getTargetAngle(DoubleSupplier distanceSupplier){
      double distance = distanceSupplier.getAsDouble();
      return -1.094 * Math.pow(distance, 2) + 9.854 * distance + 6.827;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot angle", getPosition());
    // This method will be called once per scheduler run
  }
}
