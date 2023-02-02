package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface GlobalDebug {
    void debugInit(ShuffleboardTab tab);

    void debugPeriodic(ShuffleboardTab tab);
}
