// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomous.AutoFactory;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.AutoRollerGripper;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Funnel.FunnelState;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared (subsystems,
 * commands, and trigger mappings).
 */
public class RobotContainer {
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Funnel m_funnel = new Funnel();
    private final Manipulator m_manipulator = new Manipulator();
    private final AutoRollerGripper m_autoRollerGripper = new AutoRollerGripper();
    private final Arm m_arm = new Arm();
    private final LimeLight m_limeLight = new LimeLight();

    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    private final CommandPS5Controller _driverController = new CommandPS5Controller(
            JoysticksConstants.driverPort);
    private final CommandPS5Controller _operatorController = new CommandPS5Controller(
            JoysticksConstants.operatorPort);

    private boolean _fieldRelative = true;

    private final AutoFactory _autoFactory = new AutoFactory(m_drivetrain);
    private SendableChooser<Command> chooser;

    private GlobalDebuggable[] debuggedObjects = {}; // add all subsystems that uses GlobalDebug

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_compressor.enableDigital();

        this.chooser = new SendableChooser<Command>();
        initChooser();
        // Configure the trigger bindings
        configureBindings();

        m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.drive(
                _driverController.getLeftY() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                _driverController.getLeftX() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                _driverController.getCombinedAxis() * DrivetrainConstants.maxRotationSpeedRadPerSec,
                _fieldRelative), m_drivetrain));
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        _driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field relative mode

        _driverController.share().onTrue(
                new InstantCommand(m_drivetrain::resetYaw)); // toggle field relative mode

        /* Operator triggers */
        // Collect sequence
        _operatorController.L1().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD),
                                m_funnel.setFunnelStateCommand(FunnelState.COLLECT),
                                m_arm.getSetStateCommand(ArmState.COLLECT)),
                        m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN),
                        new WaitUntilCommand(m_manipulator::isHoldingGamePiece),
                        m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD)));

        // Drive arm state sequence
        _operatorController.triangle().onTrue(
                Commands.sequence(
                        new ConditionalCommand(
                                m_funnel.setFunnelStateCommand(FunnelState.INSTALL),
                                new InstantCommand(),
                                m_manipulator::isHoldingGamePiece),
                        m_arm.getSetStateCommand(ArmState.DRIVE),
                        m_funnel.setFunnelStateCommand(FunnelState.CLOSED)));

        // Set arm to scoring pos
        _operatorController.circle().onTrue(m_arm.getSetStateCommand(ArmState.MID_CONE));
        _operatorController.square().onTrue(m_arm.getSetStateCommand(ArmState.MID_CUBE));
        _operatorController.cross().onTrue(m_arm.getSetStateCommand(ArmState.LOW));

        // Install GP
        _operatorController.R1().onTrue(m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN));

    }

    private void addToChooser(String pathName) {
        this.chooser.addOption(pathName, _autoFactory.createAuto(m_drivetrain, pathName));
    }

    private void initChooser() {
        SmartDashboard.putData("autonomous", this.chooser);
        addToChooser("1-gp-engage");
        addToChooser("1-gp-leaveCommunity");
        addToChooser("bot-2-gp-engage");
        addToChooser("bot-2-gp");
        addToChooser("bot-3-gp-engage");
        addToChooser("bot-3-gp");
    }

    /**
     * Called when we disable the robot to make sure nothing moves after we enable
     */
    public void stop() {
        m_autoRollerGripper.stop();
        m_arm.stop();
    }

    public void updateTelemetry() {
        m_drivetrain.updateTelemetry();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.chooser.getSelected();
    }

    public void debugInit(ShuffleboardTab tab) {
        for (GlobalDebuggable i : debuggedObjects) {
            i.debugInit(tab);
        }
    }

    public void debugPeriodic(ShuffleboardTab tab) {
        for (GlobalDebuggable i : debuggedObjects) {
            i.debugPeriodic(tab);
        }
    }
}
