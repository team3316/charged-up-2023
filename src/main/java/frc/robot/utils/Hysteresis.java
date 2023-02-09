package frc.robot.utils;

public class Hysteresis {
    private double _offHysteresis;
    private double _onHysteresis;
    private boolean _value;

    public Hysteresis(double bottomThreshold, double hysteresis) {
        _onHysteresis = bottomThreshold;
        _offHysteresis = bottomThreshold - hysteresis;
    }

    public boolean update(double value) {
        if (value > _onHysteresis) {
            _value = true;
        }
        if (value < _offHysteresis) {
            _value = false;
        }
        return _value;
    }

}
