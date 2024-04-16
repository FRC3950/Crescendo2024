package lib.system.pid;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import java.util.Objects;

/**
 * For use in 2025
 * */
public class TalonConfig {

    public final TalonFX motor;
    public final VelocityVoltage velVoltage = new VelocityVoltage(0);

    public PidConfig pid;
    public MotionMagicConfig motionMagicConfig;

    public TalonConfig(int canId, String canBus, PidConfig pid){
        this.motor = new TalonFX(canId, canBus);
        this.pid = pid;

        var configs = new Slot0Configs();

        velVoltage.Slot = 0;

        configs.kP = pid.kP();
        configs.kI = pid.kI();
        configs.kD = pid.kD();
        configs.kV = pid.kV();
        configs.kS = pid.kS();
        configs.kG = pid.kG();

        motor.getConfigurator().apply(configs);
    }

    public TalonConfig(int canId, String canBus, PidConfig pid, MotionMagicConfig motionMagic){
        this(canId, canBus, pid);
        this.motionMagicConfig = motionMagic;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        TalonConfig that = (TalonConfig) o;

        if (!Objects.equals(motor.getDeviceID(), that.motor.getDeviceID())) return false;
        if (!Objects.equals(pid, that.pid)) return false;
        return Objects.equals(motionMagicConfig, that.motionMagicConfig);
    }
}
