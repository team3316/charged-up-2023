package frc.robot.motors;

public class PIDFGains {
    public final double kP, kI, kD, kF, outputRange, iZone;

    public PIDFGains(double kP) {
        this(kP, 0, 0, 0, 1.0);
    }

    public PIDFGains(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 1.0);
    }

    public PIDFGains(double kP, double kI, double kD, double kF, double outputRange) {
        this(kP, kI, kD, kF, outputRange, 0);
    }

    public PIDFGains(double kP, double kI, double kD, double kF, double outputRange, double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.outputRange = outputRange;
        this.iZone = iZone;
    }
}
