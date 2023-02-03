package frc.robot.humanIO;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public enum ShuffleboardTabs {
    CONFIG("config");

    public final ShuffleboardTab tab;

    private ShuffleboardTabs(String name) {
        tab = Shuffleboard.getTab(name);
    }
}
