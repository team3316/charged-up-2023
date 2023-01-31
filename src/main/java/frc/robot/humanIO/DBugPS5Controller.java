package frc.robot.humanIO;

import frc.robot.constants.JoysticksConstants;

public class DBugPS5Controller extends PS5Controller {
    public DBugPS5Controller(int port) {
        super(port);
    }

    private double calculateDeadband(double value, double other) {
        return (Math.abs(value) > JoysticksConstants.deadBand
                || Math.abs(other) > JoysticksConstants.deadBand)
                        ? value
                        : 0;
    }

    private double squareInputs(double input) {
        return Math.copySign(input * input, input);
    }

    public double getCombinedTriggers() {
        double leftAxis = (this.getL2Axis() + 1) / 2;
        double rightAxis = (this.getR2Axis() + 1) / 2;

        if (leftAxis == 0.5 && rightAxis == 0.5)
            return 0; // joystick disconnected

        return leftAxis > rightAxis ? squareInputs(leftAxis) : squareInputs(-rightAxis);
    }

    @Override
    public double getLeftX() {
        return squareInputs(calculateDeadband(-super.getLeftX(), -super.getLeftY()));
    }

    @Override
    public double getLeftY() {
        return squareInputs(calculateDeadband(-super.getLeftY(), -super.getLeftX()));
    }

    @Override
    public double getRightX() {
        return squareInputs(calculateDeadband(-super.getRightX(), -super.getRightY()));
    }

    @Override
    public double getRightY() {
        return squareInputs(calculateDeadband(-super.getRightY(), -super.getRightX()));
    }
}
