// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pipeLine;

    /** Creates a new LimeLight. */
    public LimeLight() { // CR: add a way to send the config to the limelight trough code
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeLightTable.getEntry("tx");
        ty = limeLightTable.getEntry("ty");
        hasTarget = limeLightTable.getEntry("tv");
        pipeLine = limeLightTable.getEntry("pipeline");
    }

    public boolean hasTarget() {
        return hasTarget.getDouble(0) == 1;
    }

    public double getXAngle() {
        return LimelightConstants.AlignLimeLightToPigeonPhase * tx.getDouble(0);
    }

    public double getYAngle() {
        return ty.getDouble(0);
    }

    public double getFieldTY() {
        return new Translation2d(getXAngle(), getYAngle()).rotateBy(LimelightConstants.limelightRotationsOffset).getY();
    }

    public double getFieldTX() {
        return new Translation2d(getXAngle(), getYAngle()).rotateBy(LimelightConstants.limelightRotationsOffset).getX();
    }

    public void setPipeLine(double id) {
        pipeLine.setNumber(id);
    }
}
