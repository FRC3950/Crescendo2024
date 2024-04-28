package lib.pid;

public record PidConfig(double kP, double kI, double kD, double kV, double kS, double kG, boolean gIsCosine) {
}
