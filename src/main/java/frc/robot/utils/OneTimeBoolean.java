package frc.robot.utils;

public class OneTimeBoolean {
    private boolean _triggered;
    private boolean _value;

    public boolean update(boolean value) {
        _value = value;
        return value;
    }

    public boolean firstTime() {
        if (_triggered || !_value)
            return false;
        _triggered = true;
        return true;

    }
}