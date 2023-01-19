// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Backward extends SequentialCommandGroup {
  /** Creates a new backward. */
  public Backward(Drivetrain drivetrain) {
    FollowTrajectory followTrajectory = new FollowTrajectory(drivetrain, "backward");

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        followTrajectory.getResetOdometryCommand(),
        followTrajectory.getFollowTrajectoryCommand());
  }
}
