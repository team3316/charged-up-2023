// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SpinAndMoveByInput extends CommandBase {
  private ProfiledPIDController xController,yController,thetaController;
  private Drivetrain _drivetrain;
  private Pose2d _goal;

  /** Creates a new SpinAndMoveByInput. */
  public SpinAndMoveByInput(Drivetrain drivetrainInput, Pose2d goal) {
    xController = new ProfiledPIDController(1, 0, 0,
     new Constraints(2, 2));
    yController = new ProfiledPIDController(1, 0, 0,
     new Constraints(2, 2));
    thetaController = new ProfiledPIDController(1, 0, 0,
     new Constraints(1.5, 1.5));
    
    _drivetrain = drivetrainInput;
    _goal = goal;
//TODO to small??
    xController.setTolerance(0.02, 0.2);
    yController.setTolerance(0.02, 0.2);
    thetaController.setTolerance(0.02, 0.2);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset(_drivetrain.getPose().getX());
    yController.reset(_drivetrain.getPose().getY());
    thetaController.reset(_drivetrain.getPose().getRotation().getRadians());

    xController.setGoal(_goal.getX()+_drivetrain.getPose().getX());
    yController.setGoal(_goal.getY()+_drivetrain.getPose().getY());
    thetaController.setGoal(_goal.getRotation().getRadians()+_drivetrain.getPose().getRotation().getRadians());
    
    // SmartDashboard.putNumber("goal x",_goal.getX()+_drivetrain.getPose().getX());
    // SmartDashboard.putNumber("goal y",_goal.getY()+_drivetrain.getPose().getY());
    // SmartDashboard.putNumber("goal T",_goal.getRotation().getRadians()+_drivetrain.getPose().getRotation().getRadians());

    // SmartDashboard.putNumber("tar x",_drivetrain.getPose().getX());
    // SmartDashboard.putNumber("org x",_goal.getX());
    // SmartDashboard.putNumber("tar y",_drivetrain.getPose().getY());
    // SmartDashboard.putNumber("org y",_goal.getY());
    // SmartDashboard.putNumber("tar T",_goal.getRotation().getRadians());
    // SmartDashboard.putNumber("org T",_drivetrain.getPose().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drivetrain.drive(xController.calculate(_drivetrain.getPose().getX()),
                      yController.calculate(_drivetrain.getPose().getY()),
                  thetaController.calculate(_drivetrain.getPose().getRotation().getRadians()),
                  true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atGoal()
          && xController.atGoal()
          && yController.atGoal();
  }
}
