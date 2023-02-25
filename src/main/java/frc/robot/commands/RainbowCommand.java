package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LightsConstants;
import frc.robot.subsystems.Lights;

public class RainbowCommand extends CommandBase {

    private Lights _lights;
    private int _rainbowFirstPixelHue = 0;

    public RainbowCommand(Lights lights) {
        _lights = lights;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        for (var i = 0; i < LightsConstants.length / 2 - 1; i++) {
            final var hue = (_rainbowFirstPixelHue + (i * 180 / LightsConstants.length)) % 180;
            _lights.setPixelColor(Color.fromHSV(hue, 255, 128), i);
            if (i < 6) {
                _lights.setPixelColor(Color.fromHSV(hue, 255, 128), LightsConstants.length - i - 3);
            } else {
                _lights.setPixelColor(Color.fromHSV(hue, 255, 128), LightsConstants.length - i - 1);
            }
        }

        _rainbowFirstPixelHue += 3;
        _rainbowFirstPixelHue %= 180;

        _lights.update();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
