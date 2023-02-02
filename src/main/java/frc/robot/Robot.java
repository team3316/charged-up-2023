// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.humanIO.ShuffleboardTabs;
import frc.robot.utils.OneTimeBoolean;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private OneTimeBoolean _debug = new OneTimeBoolean();

    private GenericEntry debugWidget = ShuffleboardTabs.CONFIG.tab.add("debug", _debug)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private static double LOGGING_PERIOD_SECONDS = 0.2;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        addPeriodic(() -> {
            if (_debug.update(debugWidget.getBoolean(false))) {
                if (_debug.firstTime())
                    m_robotContainer.debugInit(ShuffleboardTabs.CONFIG.tab);

                m_robotContainer.updateTelemetry();
                m_robotContainer.debugPeriodic(ShuffleboardTabs.CONFIG.tab);
            }
        }, LOGGING_PERIOD_SECONDS);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
