package lib.system.pid;

public record MotionMagicConfig (double kA, double cruiseVel, double accel, double jerk) {

    @Override
    public boolean equals(Object other){
        if(!(other instanceof MotionMagicConfig otherConfig)){
            return false;
        }

        return kA == otherConfig.kA && cruiseVel == otherConfig.cruiseVel && accel == otherConfig.accel && jerk == otherConfig.jerk;
    }
}
