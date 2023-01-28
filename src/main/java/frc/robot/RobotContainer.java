// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.calibrations.ArmCalibration;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared (subsystems,
 * commands, and trigger mappings).
 */
public class RobotContainer {
        private final Drivetrain m_drivetrain = new Drivetrain();
        private final Manipulator m_Manipulator = new Manipulator();
        private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
        private final Arm m_arm = new Arm();

        private final CommandPS5Controller _driverController = new CommandPS5Controller(
                        JoysticksConstants.driverPort);

        private boolean _fieldRelative = true;

        private final ArmCalibration m_armCalibration = new ArmCalibration(m_arm);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_compressor.enableDigital();

                // Configure the trigger bindings
                configureBindings();

                m_drivetrain.setDefaultCommand(
                                new RunCommand(
                                                () -> m_drivetrain.drive(
                                                                _driverController.getLeftY()
                                                                                * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                                                _driverController.getLeftX()
                                                                                * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                                                _driverController.getCombinedAxis()
                                                                                * DrivetrainConstants.maxRotationSpeedRadPerSec,
                                                                _fieldRelative),
                                                m_drivetrain));
        }

        /**
         * Use this method to define your trigger->command mappings.
         */
        private void configureBindings() {
                _driverController.options().onTrue(
                                new InstantCommand(() -> _fieldRelative = !_fieldRelative)); // toggle field relative
                                                                                             // mode

                _driverController.share().onTrue(
                                new InstantCommand(m_drivetrain::resetYaw)); // toggle field relative mode

                _driverController.triangle().onTrue(
                                m_Manipulator.setManipulatorStateCommand(Manipulator.ManipulatorState.CONE_HOLD));
                _driverController.square().onTrue(
                                m_Manipulator.setManipulatorStateCommand(Manipulator.ManipulatorState.CUBE_HOLD));
                _driverController.cross().onTrue(
                                m_Manipulator.setManipulatorStateCommand(Manipulator.ManipulatorState.OPEN));

                _driverController.circle().onTrue(
                                m_armCalibration.endSequence());

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new InstantCommand();
        }

        public Command getArmCalibrationCommand() {
                return new ArmCalibration(m_arm);
        }
}
