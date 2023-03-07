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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomous.AutoFactory;
import frc.robot.autonomous.GyroEngage;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.ArmFunnelSuperStructure;
import frc.robot.subsystems.AutoRollerGripper;
import frc.robot.subsystems.AutoRollerGripper.FolderState;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Funnel.FunnelState;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.IRSensorState;
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

    private final AutoFactory _autoFactory = new AutoFactory(m_drivetrain, m_autoRollerGripper);
    private SendableChooser<CommandBase> chooser;

    private GlobalDebuggable[] debuggedObjects = {}; // add all subsystems that uses GlobalDebug

    private boolean _scoreMidCube;

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

        setCubeInternalState();
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
        _driverController.cross()
                .whileTrue(new InstantCommand(() -> m_drivetrain.restartControllers(), m_drivetrain).andThen(
                        new RunCommand(() -> m_drivetrain.driveByVisionControllers(m_limeLight.getFieldTX(),
                                m_limeLight.getFieldTY()), m_drivetrain)));

        /* Operator triggers */
        // Collect sequence
        _operatorController.L1().onTrue(
                Commands.sequence(
                        m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT, FunnelState.COLLECT),
                        m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN),
                        new WaitUntilCommand(m_manipulator::isHoldingGamePiece),
                        new WaitCommand(0.5),
                        m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT, FunnelState.CLOSED),
                        m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD)));

        // Drive arm state sequence
        _operatorController.povUp().onTrue(
                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.DRIVE, FunnelState.CLOSED)
                        .beforeStarting(m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD)));

        // Set cube as wanted GP
        _operatorController.square().onTrue(new InstantCommand(() -> setCubeInternalState()));

        // Set cone as wanted GP
        _operatorController.circle().onTrue(new InstantCommand(() -> setConeInternalState()));

        // Score sequences
        _operatorController.R2()
                .onTrue(m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.LOW, FunnelState.CLOSED)
                        .beforeStarting(m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD)));

        _operatorController.R1().onTrue(
                new ConditionalCommand(
                        m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.MID_CUBE, FunnelState.CLOSED)
                                .beforeStarting(m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD)),
                        m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.MID_CONE, FunnelState.CLOSED)
                                .beforeStarting(m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD)),
                        () -> _scoreMidCube));

        // Install GP
        _operatorController.L2().onTrue(m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN));

        // Go to collect state sequence
        _operatorController.povDown().onTrue(
                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT, FunnelState.CLOSED)
                        .andThen(m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN)));

        _driverController.circle().whileTrue(
                new InstantCommand(() -> m_drivetrain.setKeepHeading(DrivetrainConstants.collectAngle)).andThen(
                        new RunCommand(() -> m_drivetrain.driveAndKeepHeading(
                                _driverController.getLeftY() *
                                        SwerveModuleConstants.freeSpeedMetersPerSecond,
                                _driverController.getLeftX() *
                                        SwerveModuleConstants.freeSpeedMetersPerSecond),
                                m_drivetrain)));

        _operatorController.share().onTrue(m_autoRollerGripper.getIntakeCommand());
        _operatorController.options().onTrue(m_autoRollerGripper.getEjectCommand());
        _driverController.povDown().onTrue(m_autoRollerGripper.getFoldCommand(FolderState.OUT));
        _driverController.povUp().onTrue(m_autoRollerGripper.getFoldCommand(FolderState.IN));

    }

    private void setCubeInternalState() {
        this._scoreMidCube = true;
        m_limeLight.setPipeLine(LimelightConstants.pipeLineAprilTags);
        SmartDashboard.putBoolean("target GP", this._scoreMidCube);
        m_drivetrain.setVisionAprilPID();
        m_manipulator.setIRSensorState(IRSensorState.CUBE);
    }

    private void setConeInternalState() {
        this._scoreMidCube = false;
        m_limeLight.setPipeLine(LimelightConstants.pipeLineRetroReflective);
        SmartDashboard.putBoolean("target GP", this._scoreMidCube);
        m_drivetrain.setVisionRetroPID();
        m_manipulator.setIRSensorState(IRSensorState.CONE);
    }

    private void addToChooser(String pathName) {
        this.chooser.addOption(pathName, _autoFactory.createAuto(pathName));
    }

    private void initChooser() {
        SmartDashboard.putData("autonomous", this.chooser);
        // addToChooser("engage");
        // addToChooser("1-gp-engage");
        addToChooser("1-gp-leaveCommunity");
        // addToChooser("bot-2-gp-engage");
        addToChooser("bot-2-gp");
        // addToChooser("bot-3-gp-engage");
        // addToChooser("bot-3-gp");

        this.chooser.addOption("nothing", new InstantCommand());

        // only cube
        this.chooser.addOption("cube", getAutoCubeSequence());

        // cube engage
        this.chooser.addOption("cube-engage-gyro", getAutoCubeSequence().andThen(getEngageSequence()));
        // only engage
        this.chooser.addOption("engage-gyro", getEngageSequence());

        // taxi
        this.chooser.addOption("taxi", _autoFactory.createAuto("engage-gyro"));

        // cube taxi
        this.chooser.addOption("cube-taxi", getAutoCubeSequence().andThen(_autoFactory.createAuto("engage-gyro")));
    }

    /**
     * Called when we disable the robot to make sure nothing moves after we enable
     */
    public void stop() {
        m_autoRollerGripper.stop();
        m_ArmFunnelSuperStructure.stop();
        m_drivetrain.calibrateSteering();
    }

    public void subsystemsInit() {
        m_ArmFunnelSuperStructure.init();
    }

    public void calibrateSteering() {
        m_drivetrain.calibrateSteering();
    }

    public void updateTelemetry() {
        m_drivetrain.updateTelemetry();
    }

    private CommandBase getEngageSequence() {
        return Commands.sequence(
                _autoFactory.createAuto("engage-gyro"),
                new GyroEngage(m_drivetrain, 0.5, -5, true),
                new RunCommand(() -> m_drivetrain.drive(0, -0.1, 0, false)).withTimeout(0.2),
                new GyroEngage(m_drivetrain, -0.12, 5, false),
                new RunCommand(() -> m_drivetrain.drive(0, -0.1, 0, false)).withTimeout(0.2));
    }

    private CommandBase getAutoCubeSequence() {
        return Commands.sequence(new InstantCommand(() -> this.setCubeInternalState()),
                m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN),
                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT,
                        FunnelState.COLLECT),
                new WaitUntilCommand(m_manipulator::isHoldingGamePiece),
                m_ArmFunnelSuperStructure.getSetStateCommand(ArmState.COLLECT, FunnelState.CLOSED),
                new WaitCommand(0.5),
                m_manipulator.setManipulatorStateCommand(ManipulatorState.HOLD),
                m_ArmFunnelSuperStructure.overrideCommand(),
                m_manipulator.setManipulatorStateCommand(ManipulatorState.OPEN));
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
