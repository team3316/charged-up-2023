package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LightsConstants;
import frc.robot.subsystems.Lights;

public class BlinkCommand extends CommandBase {

    private Lights _lights;
    private Color _color;

    private double _lastChange = 0;
    private boolean _on = false;
    private double _interval = LightsConstants.blinkInterval;

    public BlinkCommand(Lights lights, Color color) {
        _lights = lights;
        _color = color;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double timestamp = Timer.getFPGATimestamp();
        if (timestamp - _lastChange > _interval) {
            _on = !_on;
            _lastChange = timestamp;
            if (_on) {
                _lights.setColor(_color);
            } else {
                _lights.setColor(Color.kBlack);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
