package lib.pid;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

/**
 * For use in 2025
 * */
public class TalonConfig {

    public final TalonFX motor;

    public final VelocityVoltage velVoltage = new VelocityVoltage(0);
    public final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

    public final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    public final Slot0Configs configs = new Slot0Configs();

    public TalonConfig(int canId, String canBus, PidConfig pid){
        this.motor = new TalonFX(canId, canBus);

        velVoltage.Slot = 0;
        mmVoltage.Slot = 0;

        configs.kP = pid.kP();
        configs.kI = pid.kI();
        configs.kD = pid.kD();
        configs.kV = pid.kV();
        configs.kS = pid.kS();
        configs.kG = pid.kG();

        configs.GravityType = pid.gIsCosine() ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;

        motor.getConfigurator().apply(configs);
    }

    public TalonConfig(int canId, String canBus, PidConfig pid, MotionMagicConfig motionMagicConfig){
        this(canId, canBus, pid);

        mmConfigs.withMotionMagicAcceleration(motionMagicConfig.accel());
        mmConfigs.withMotionMagicCruiseVelocity(motionMagicConfig.cruiseVel());
        mmConfigs.withMotionMagicJerk(motionMagicConfig.jerk());

        motor.getConfigurator().apply(configs);
    }

    public TalonConfig withFollower(int followerId, String followerBus, boolean isInverted){

        var follower = new TalonFX(followerId, followerBus);

        follower.getConfigurator().apply(configs);
        follower.setControl(new Follower(motor.getDeviceID(), isInverted));

        return this;
    }

    public boolean isAtForwardLimit(){
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean isAtReverseLimit() {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public void setMotionMagicConfig(MotionMagicConfig mmConfig){
        mmConfigs.withMotionMagicCruiseVelocity(mmConfig.cruiseVel());
        mmConfigs.withMotionMagicAcceleration(mmConfig.accel());
        mmConfigs.withMotionMagicJerk(mmConfig.jerk());

        motor.getConfigurator().apply(configs);
    }


}
