package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {

    private final TalonFX motor1 = new TalonFX(Constants.Climber.motor1Id);
    private final TalonFX motor2 = new TalonFX(Constants.Climber.motor2Id);

    public Climber() {}

    public Command climbCommand(DoubleSupplier yAxisPercentage) {
        return Commands.runOnce(() -> setMotorVoltage(yAxisPercentage), this);
    }

    private double getTargetVoltage(DoubleSupplier yAxisPercentage, double motorPosition){
        var percent = yAxisPercentage.getAsDouble();

        if(motorPosition < 3.95 && percent > 0.15){
            return -yAxisPercentage.getAsDouble() * 6;
        }
        else if(percent < -0.15){
            return yAxisPercentage.getAsDouble() * -8;
        }

        return 0;
    }

    private void setMotorVoltage(DoubleSupplier yAxisPercentage){
        motor1.getPosition().refresh();
        motor2.getPosition().refresh();

        motor1.setVoltage(-getTargetVoltage(yAxisPercentage, Math.abs(motor1.getPosition().getValueAsDouble())));
        motor2.setVoltage(-getTargetVoltage(yAxisPercentage, Math.abs(motor2.getPosition().getValueAsDouble())));
    }

    @Override
    public void periodic() {}
}
