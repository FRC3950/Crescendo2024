package frc.robot.supersystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TargetVelocity {
    public TalonFX motor;
    public double velocity;

    public double kP;
    public double kV;

    public VelocityVoltage velVoltage = new VelocityVoltage(0);

    public TargetVelocity(TalonFX motor, double velocity, double kP, double kV){
        this.motor = motor; 
        this.velocity = velocity;

        this.kP = kP;
        this.kV = kV;

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = kP;
        slot0Configs.kV = kV;

        velVoltage.Slot = 0;
    }

    /**
     * Constructor for two-motor systems. 
     * @param leader The leader motor.
     * @param follower Follows (and is inverted from) leader motor.
     * @param velocity The target velocity for both motors.
     * @param kP The kP of the velocity profile for both motors. 
     * @param kV The kV of the velocity profile for both motors.
     */
    public TargetVelocity(TalonFX leader, TalonFX follower, double velocity, double kP, double kV){
        this.motor = leader;

        this.kP = kP;
        this.kV = kV;

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = kP;
        slot0Configs.kV = kV;

        velVoltage.Slot = 0;

        follower.setControl(new Follower(motor.getDeviceID(), true));
    }
}
