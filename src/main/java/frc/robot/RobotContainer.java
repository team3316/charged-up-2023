// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutoFactory;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AutoRollerGripper;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.DynamicCommand;

/**
 * This class is where the bulk of the robot should be declared (subsystems,
 * commands, and trigger mappings).
 */
public class RobotContainer {
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Funnel m_Funnel = new Funnel();
    private final Manipulator m_Manipulator = new Manipulator();
    private final AutoRollerGripper m_autoRollerGripper = new AutoRollerGripper();
    private final Arm m_arm = new Arm();

    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    private final CommandPS5Controller _driverController = new CommandPS5Controller(
            JoysticksConstants.driverPort);

    private boolean _fieldRelative = true;

    private final AutoFactory _autoFactory = new AutoFactory(m_drivetrain);
    private SendableChooser<CommandBase> chooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_compressor.enableDigital();

        this.chooser = new SendableChooser<CommandBase>();
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

        _driverController.cross().onTrue(
                new InstantCommand(() -> m_drivetrain.setModulesAngle(SmartDashboard.getNumber("module angles", 0))));

        _driverController.PS()
                .whileTrue(Commands.sequence(
                        new DynamicCommand(() -> getAutonomousCommand(), m_drivetrain),
                        new InstantCommand(() -> m_drivetrain.drive(0, 0, 0, false)),
                        new WaitCommand(0.5),
                        _autoFactory.createAuto(m_drivetrain, "engage-2"))
                        .finallyDo((interrupted) -> m_drivetrain.setModulesAngle(90)));
    }

    private void addToChooser(String pathName) {
        this.chooser.addOption(pathName, _autoFactory.createAuto(m_drivetrain, pathName));
    }

    private void initChooser() {
        SmartDashboard.putData("autonomous", this.chooser);
        addToChooser("engage");
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
        m_drivetrain.calibrateSteering();
    }

    public void updateTelemetry() {
        m_drivetrain.updateTelemetry();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public CommandBase getAutonomousCommand() {
        return this.chooser.getSelected();
    }
}
