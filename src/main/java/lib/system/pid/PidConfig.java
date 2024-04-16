package lib.system.pid;

public record PidConfig(double kP, double kI, double kD, double kV, double kS, double kG, boolean gIsCosine) {
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        PidConfig pidConfig = (PidConfig) o;

        if (Double.compare(kP, pidConfig.kP) != 0) return false;
        if (Double.compare(kI, pidConfig.kI) != 0) return false;
        if (Double.compare(kD, pidConfig.kD) != 0) return false;
        if (Double.compare(kV, pidConfig.kV) != 0) return false;
        if (Double.compare(kS, pidConfig.kS) != 0) return false;
        if (Double.compare(kG, pidConfig.kG) != 0) return false;
        return gIsCosine == pidConfig.gIsCosine;
    }
}
