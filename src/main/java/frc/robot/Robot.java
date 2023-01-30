// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.humanIO.ShuffleboardTabs;

public class Robot extends TimedRobot {

    private static HashMap<String, Boolean> _debugBoolean = new HashMap<String, Boolean>();
    private static HashMap<String, Double> _debugDouble = new HashMap<String, Double>();
    private static HashMap<String, String> _debugString = new HashMap<String, String>();

    private static HashMap<String, Boolean> _debugBooleanSingle = new HashMap<String, Boolean>();
    private static HashMap<String, Double> _debugDoubleSingle = new HashMap<String, Double>();
    private static HashMap<String, String> _debugStringSingle = new HashMap<String, String>();

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private boolean _debug = false;
    private boolean _debugCalled = false;

    private GenericEntry debugWidget = ShuffleboardTabs.CONFIG.tab.add("debug", _debug)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private static double LOGGING_PERIOD_SECONDS = 0.2;

    /**
     * Adds the element to SmartDashboard when debug switch is enabled
     * 
     * @param id:    SmartDashboard's key
     * @param value: SmartDashboard's value
     */
    public static void insertBooleanToDebug(String id, boolean value, boolean singleCall) {
        if (singleCall)
            _debugBooleanSingle.put(id, value);
        _debugBoolean.put(id, value);
    }

    public static void insertStringToDebug(String id, String value, boolean singleCall) {
        if (singleCall)
            _debugStringSingle.put(id, value);
        _debugString.put(id, value);
    }

    public static void insertNumberToDebug(String id, double value, boolean singleCall) {
        if (singleCall)
            _debugDoubleSingle.put(id, value);
        _debugDouble.put(id, value);
    }

    private void checkDebugPeriodic() {
        if (!_debug)
            return;
        if (!_debugCalled) {
            _debugBooleanSingle.forEach((String id, Boolean value) -> SmartDashboard.putBoolean(id, value));
            _debugStringSingle.forEach((String id, String value) -> SmartDashboard.putString(id, value));
            _debugDoubleSingle.forEach((String id, Double value) -> SmartDashboard.putNumber(id, value));
            _debugCalled = true;
        }
        _debugBoolean.forEach((String id, Boolean value) -> SmartDashboard.putBoolean(id, value));
        _debugDouble.forEach((String id, Double value) -> SmartDashboard.putNumber(id, value));
        _debugString.forEach((String id, String value) -> SmartDashboard.putString(id, value));
    }

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        addPeriodic(() -> {
            if (_debug)
                m_robotContainer.updateTelemetry();
        }, LOGGING_PERIOD_SECONDS);
    }

    @Override
    public void robotPeriodic() {
        _debug = debugWidget.getBoolean(_debug);

        CommandScheduler.getInstance().run();
        checkDebugPeriodic();
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
