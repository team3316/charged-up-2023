package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.BlinkCommand;
import frc.robot.commands.RainbowCommand;
import frc.robot.constants.LightsConstants;
import frc.robot.constants.LightsConstants.RobotColorState;

public class Lights extends SubsystemBase {
    private AddressableLED _led;
    private AddressableLEDBuffer _mainBuffer;

    public Lights() {
        this._led = new AddressableLED(LightsConstants.port);
        this._mainBuffer = new AddressableLEDBuffer(LightsConstants.length);

        this._led.setLength(this._mainBuffer.getLength());

        this._led.setData(this._mainBuffer);
        this._led.start();
    }

    public void setColor(Color color) {
        for (int i = 0; i < this._mainBuffer.getLength(); i++) {
            this._mainBuffer.setLED(i, color);
        }

        this._led.setData(this._mainBuffer);
    }

    public void setPixelColor(Color color, int i) {
        this._mainBuffer.setLED(i, color);
    }

    public void update() {
        this._led.setData(this._mainBuffer);
    }

    public CommandBase getColorCommand(RobotColorState state) {
        CommandBase command = new InstantCommand();
        switch (state.pattern) {
            case SOLID:
                command = new InstantCommand(() -> setColor(state.color), this);
            case BLINK:
                command = new BlinkCommand(this, state.color);
            case RAINBOW:
                command = new RainbowCommand(this);
        }
        return new ProxyCommand(command.ignoringDisable(true));
    }
}
