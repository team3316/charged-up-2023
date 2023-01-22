// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    public LimeLight() { // CR: add a way to send the config to the limelight trough code
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeLightTable.getEntry("tx");
        ty = limeLightTable.getEntry("ty");
    }

    public double getAngle() {
        return -1 * tx.getDouble(0); // negative because LL and pigeon have opposite sensor fazzes
    }

    // height dif between goal and limelight devided by tan of limelight y angle.
    public double getDist() {
        return LimelightConstants.limeLightGoalHeightOffset
                / Math.tan(Math.toRadians(
                        ty.getDouble(0) + LimelightConstants.limelightAngleFromFloorDegs));
    }

    public Translation2d getTrans(Rotation2d gyroRotation) {
        // CR: Use Translation2d
        return new Translation2d(
                getDist(), Rotation2d.fromDegrees(getAngle()).plus(gyroRotation));

    }

}
