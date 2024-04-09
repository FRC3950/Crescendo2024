package lib.system;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TargetPosition {

    public final TalonFX motor;
    public final DoubleSupplier initalPosition;

    public final double kP;
    public final double kV;
    public final double kG;

    public final MotionMagicVoltage mmVoltage;
    public final Slot0Configs slot0Configs;

    public TargetPosition(TalonFX motor, DoubleSupplier initalPosition, double kP, double kV, double kG, boolean kA){
        this.motor = motor;
        this.initalPosition = initalPosition;

        mmVoltage = new MotionMagicVoltage(initalPosition.getAsDouble());

        this.kP = kP;
        this.kV = kV;
        this.kG = kG;

        slot0Configs = new Slot0Configs();

        slot0Configs.kP = kP;
        slot0Configs.kV = kV;
        slot0Configs.kG = kG;

        if(kA){
            slot0Configs.kA = 1.0;
        }

        mmVoltage.Slot = 0;

        motor.getConfigurator().apply(slot0Configs);
    }
}
