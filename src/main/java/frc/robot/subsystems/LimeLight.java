// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;

public class LimeLight extends SubsystemBase {
    NetworkTable limeLightTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;

    /** Creates a new LimeLight. */
    public LimeLight() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeLightTable.getEntry("tx");
        ty = limeLightTable.getEntry("ty");
    }

    public double getAngle() {
      return -1*tx.getDouble(0); //negative because LL and pigeon have opposite sensor fazzes
    }

    //height dif between goal and limelight devided by tan of limelight y angle.
    public double getDist() {
      return LimelightConstants.limeLightToLowGoalMeters
      /Math.tan(Math.toRadians(
        ty.getDouble(0)+LimelightConstants.limelightAngleFromFloorDegs
        )
      );
    }

    public Pose2d getPose() {
      return new Pose2d(
        getDist()*Math.cos(Math.toRadians(getAngle())),
        getDist()*Math.sin(Math.toRadians(getAngle())),
        new Rotation2d());
    }

    public Pose2d getPose(double rotateRads) {
      return new Pose2d(
        getDist()*Math.cos(Math.toRadians(getAngle())+rotateRads),
        getDist()*Math.sin(Math.toRadians(getAngle())+rotateRads),
        new Rotation2d());
    }
    

    

}
