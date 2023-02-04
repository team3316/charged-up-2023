package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface GlobalDebuggable {
    void debugInit(ShuffleboardTab tab);

    void debugPeriodic(ShuffleboardTab tab);
}
