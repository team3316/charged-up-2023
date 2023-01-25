// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        return new Translation2d(getDist(), Rotation2d.fromDegrees(getAngle()).plus(gyroRotation))
                .minus(new Translation2d(0, LimelightConstants.installDistanceFromLowGoalMeters));

    }

    public int getClosest() {
        int targets = (int)limeLightTable.getEntry("tv").getDouble(0);
        switch (targets){
            case 1:
                return (int)limeLightTable.getEntry("td").getDouble(0);
            case 2:
                
            
                default:
                return 0;

        }
    }

    public Pose2d getPos() {
        double[] poseComponents = limeLightTable.getEntry("botpose").getDoubleArray(new double[6]);
        if (poseComponents.length == 0)
            return new Pose2d();

        return new Pose2d(
                poseComponents[0],
                poseComponents[1],
                new Rotation2d(
                        poseComponents[3],
                        poseComponents[4]));

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("limelight position", this.getPos().toString());
        SmartDashboard.putNumber("target", limeLightTable.getEntry("tv").getDouble(0));
    }

}
