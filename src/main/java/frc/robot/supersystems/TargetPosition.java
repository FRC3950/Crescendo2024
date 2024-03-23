package frc.robot.supersystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class TargetPosition {

    public final TalonFX motor;
    public final DoubleSupplier initalPosition;

    public final double kP;
    public final double kV;
    public final double kG;

    public final MotionMagicVoltage mmVoltage;

    public TargetPosition(TalonFX motor, DoubleSupplier initalPosition, double kP, double kV, double kG){
        this.motor = motor;
        this.initalPosition = initalPosition;

        mmVoltage = new MotionMagicVoltage(initalPosition.getAsDouble());

        this.kP = kP;
        this.kV = kV;
        this.kG = kG;

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = kP;
        slot0Configs.kV = kV;
        slot0Configs.kG = kG;

        mmVoltage.Slot = 0;
    }

    public TargetPosition(TalonFX motor1, TalonFX motor2, DoubleSupplier initialPosition, double kP, double kV, double kG, boolean motorsAreInverted){
        this.motor = motor1;
        this.initalPosition = initialPosition;

        mmVoltage = new MotionMagicVoltage(initialPosition.getAsDouble());

        this.kP = kP;
        this.kV = kV;
        this.kG = kG;

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = kP;
        slot0Configs.kV = kV;
        slot0Configs.kG = kG;

        mmVoltage.Slot = 0;

        motor2.setControl(new Follower(motor1.getDeviceID(), motorsAreInverted));
    }
}
