package frc.robot.utils;

public class Hysteresis {
    private double _topHysteresis;
    private double _bottomHysteresis;
    private boolean _value;

    public Hysteresis(double bottomThreshold, double hysteresis) {
        _bottomHysteresis = bottomThreshold;
        _topHysteresis = bottomThreshold + hysteresis;
    }

    public boolean update(double value) {
        if (value > _topHysteresis) {
            _value = true;
        }
        if (value < _bottomHysteresis) {
            _value = false;
        }
        return _value;
    }

}
