// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomous.AutoFactory;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.ArmFunnelSuperStructure;
import frc.robot.subsystems.AutoRollerGripper;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Funnel.FunnelPosition;
import frc.robot.subsystems.Funnel.FunnelRollersState;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.GlobalDebuggable;

/**
 * This class is where the bulk of the robot should be declared (subsystems,
 * commands, and trigger mappings).
 */
public class RobotContainer {
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Manipulator m_manipulator = new Manipulator();
    private final AutoRollerGripper m_autoRollerGripper = new AutoRollerGripper();

    private final ArmFunnelSuperStructure m_ArmFunnelSuperStructure = new ArmFunnelSuperStructure(new Arm(),
            new Funnel());

    private final LimeLight m_limeLight = new LimeLight();

    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    private final CommandPS5Controller _driverController = new CommandPS5Controller(
            JoysticksConstants.driverPort);
    private final CommandPS5Controller _operatorController = new CommandPS5Controller(
            JoysticksConstants.operatorPort);

    private boolean _fieldRelative = true;

    private final AutoFactory _autoFactory = new AutoFactory(m_drivetrain);
    private SendableChooser<CommandBase> chooser;

    private GlobalDebuggable[] debuggedObjects = {}; // add all subsystems that uses GlobalDebug

    private boolean _scoreMidCube = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_compressor.enableDigital();
        // m_drivetrain.visionInitSDB();

        this.chooser = new SendableChooser<CommandBase>();
        initChooser();
        // Configure the trigger bindings
        configureBindings();

        m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.drive(
                _driverController.getLeftY() *
                        SwerveModuleConstants.freeSpeedMetersPerSecond,
                _driverController.getLeftX() *
                        SwerveModuleConstants.freeSpeedMetersPerSecond,
                _driverController.getCombinedAxis() *
                        DrivetrainConstants.maxRotationSpeedRadPerSec,
                _fieldRelative), m_drivetrain));

        SmartDashboard.putBoolean("target GP", this._scoreMidCube);
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        _driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field
        // relative mode

        _driverController.share().onTrue(
                new InstantCommand(m_drivetrain::resetYaw)); // toggle field relative mode

        /* align with vision target */
        _driverController.triangle()
                .whileTrue(new InstantCommand(() -> m_drivetrain.restartControllers(), m_drivetrain).andThen(
                        new RunCommand(() -> m_drivetrain.driveByVisionControllers(m_limeLight.getFieldTX(),
                                m_limeLight.getFieldTY()), m_drivetrain)));

        /* Operator triggers */
        // Collect sequence
        _operatorController.L1().onTrue(
                Commands.sequence(
                        Commands.sequence(
                                m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN),
                                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT, FunnelPosition.OPEN),
                                m_ArmFunnelSuperStructure.setFunnelRollersStateCommand(FunnelRollersState.COLLECT)),
                        new WaitUntilCommand(m_manipulator::isHoldingGamePiece),
                        Commands.sequence(

                                m_ArmFunnelSuperStructure.setFunnelRollersStateCommand(FunnelRollersState.OFF),
                                m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD),
                                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT,
                                        FunnelPosition.CLOSED))));

        // Drive arm state sequence
        _operatorController.triangle().onTrue(
                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.DRIVE, FunnelPosition.CLOSED));

        // Set cube as wanted GP
        _operatorController.circle().onTrue(new InstantCommand(() -> {
            this._scoreMidCube = true;
            m_limeLight.setPipeLine(LimelightConstants.pipeLineAprilTags);
            SmartDashboard.putBoolean("target GP", this._scoreMidCube);
            m_drivetrain.setVisionAprilPID();
        }));

        // Set cone as wanted GP
        _operatorController.square().onTrue(new InstantCommand(() -> {
            this._scoreMidCube = false;
            m_limeLight.setPipeLine(LimelightConstants.pipeLineRetroReflective);
            SmartDashboard.putBoolean("target GP", this._scoreMidCube);
            m_drivetrain.setVisionRetroPID();
        }));

        // Score sequences
        _operatorController.R2()
                .onTrue(m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.LOW, FunnelPosition.CLOSED));

        _operatorController.R1().onTrue(
                new ConditionalCommand(
                        m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.MID_CUBE, FunnelPosition.CLOSED),
                        m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.MID_CONE, FunnelPosition.CLOSED),
                        () -> _scoreMidCube));

        // Install GP
        _operatorController.L2().onTrue(m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN));

        // Go to collect state sequence
        _operatorController.cross().onTrue(
                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT, FunnelPosition.CLOSED));

    }

    private void addToChooser(String pathName) {
        this.chooser.addOption(pathName, _autoFactory.createAuto(pathName));
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
        m_ArmFunnelSuperStructure.stop();
        m_drivetrain.calibrateSteering();
    }

    public void calibrateSteering() {
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
