// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroup.backward;
import frc.robot.commandGroup.moveFoward;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final LimeLight m_LimeLight = new LimeLight();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandPS5Controller _driverController = new CommandPS5Controller(
            JoysticksConstants.driverPort);

    private boolean _fieldRelative = true;

    private SendableChooser<Command> chooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.chooser = new SendableChooser<Command>();
        initChooser();
        // Configure the trigger bindings
        configureBindings();
        SmartDashboard.putNumber("target angle", 0);
        SmartDashboard.putNumber("kP", 0);
        

        m_drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> m_drivetrain.drive(
                                _driverController.getLeftY() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                _driverController.getLeftX() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                _driverController.getCombinedAxis() * 11.5,
                                _fieldRelative),
                        m_drivetrain));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        _driverController.options().onTrue(
                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field relative mode

        _driverController.share().onTrue(
                new InstantCommand(m_drivetrain::resetYaw)); // toggle field relative mode
        _driverController.cross().onTrue(
                new InstantCommand(
                () -> m_drivetrain.getSpinToAngleCommand(Rotation2d.fromDegrees(
                        SmartDashboard.getNumber("target angle", 0))).schedule()
                )
        );
        _driverController.L3().onTrue(
                new InstantCommand(() -> m_drivetrain.getCurrentCommand().cancel()).
                andThen(new InstantCommand(
                        () -> SmartDashboard.putNumber("target angle", 0)
                ))
        );

    }

    public void initChooser() {
        this.chooser.setDefaultOption("forward", new moveFoward(m_drivetrain));
        this.chooser.addOption("backward", new backward(m_drivetrain));
        SmartDashboard.putData("autonomous", this.chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return this.chooser.getSelected();
    }
}
