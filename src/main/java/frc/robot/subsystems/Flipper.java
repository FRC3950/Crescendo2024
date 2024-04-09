// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.system.PositionController;
import lib.system.TargetPosition;

import java.util.function.DoubleSupplier;

public class Flipper extends PositionController {

    public Flipper() {
        super(
                new TargetPosition(
                        new TalonFX(Constants.Flipper.id, "CANivore"), Constants.Flipper.stowPosition,
                        Constants.Flipper.kP, Constants.Flipper.kV, Constants.Flipper.kG, false
                )
        );
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command stowCommand() {
        return Commands.runOnce(() -> setPosition(Constants.Flipper.stowPosition));
    }

    @CommandBehavior(behavior = CommandType.INSTANT)
    public Command ampCommand(DoubleSupplier pos) {
        return Commands.runOnce(() -> setPosition(pos));
    }

    @Override
    public void periodic() {
    }
}
