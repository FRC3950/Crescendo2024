package frc.robot.supersystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TargetVelocity {
    public final TalonFX motor;
    public final double targetVelocity;

    public final double kP;
    public final double kV;

    public final VelocityVoltage velVoltage = new VelocityVoltage(0);

    public TargetVelocity(TalonFX motor, double targetVelocity, double kP, double kV){
        this.motor = motor; 
        this.targetVelocity = targetVelocity;

        this.kP = kP;
        this.kV = kV;

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = kP;
        slot0Configs.kV = kV;

        velVoltage.Slot = 0;

        //motor.getConfigurator().apply(slot0Configs);
    }

    /**
     * Target velocity for two-motor leader-follower systems. 
     * @param leader The leader motor.
     * @param follower Follows leader motor.
     * @param targetVelocity The target velocity for both motors.
     * @param kP The kP of the targetVelocity profile for both motors. 
     * @param kV The kV of the targetVelocity profile for both motors.
     */
    public TargetVelocity(TalonFX leader, TalonFX follower, double targetVelocity, double kP, double kV, boolean isInverted){
        this.motor = leader;

        this.kP = kP;
        this.kV = kV;

        this.targetVelocity = targetVelocity;

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = kP;
        slot0Configs.kV = kV;

        velVoltage.Slot = 0;

        motor.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);

        follower.setControl(new Follower(motor.getDeviceID(), isInverted));
    }
}
