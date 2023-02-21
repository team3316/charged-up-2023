package frc.robot.utils;

public class Within {
    public static boolean range(double value, double target, double tol) {
        return Math.abs(target - value) <= tol;
    }
}
