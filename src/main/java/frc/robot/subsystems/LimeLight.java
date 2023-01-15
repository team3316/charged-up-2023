// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

    

}
