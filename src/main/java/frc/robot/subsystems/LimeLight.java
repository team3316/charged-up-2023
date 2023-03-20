// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pipeLine;
    private NetworkTableEntry LEDs;
    private double hDiff = 0;

    /** Creates a new LimeLight. */
    public LimeLight() { // CR: add a way to send the config to the limelight trough code
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeLightTable.getEntry("tx");
        ty = limeLightTable.getEntry("ty");
        hasTarget = limeLightTable.getEntry("tv");
        pipeLine = limeLightTable.getEntry("pipeline");
        LEDs = limeLightTable.getEntry("ledMode");
    }

    public boolean hasTarget() {
        return hasTarget.getDouble(0) == 1;
    }

    private double getXAngle() {
        return LimelightConstants.AlignLimeLightToPigeonPhase * tx.getDouble(0);
    }

    private double getYAngle() {
        return ty.getDouble(0);
    }

    public double getFieldYMeters() {
        return this.hDiff * Math.sin(Math.toRadians(this.getXAngle()))
                / Math.tan(Math.toRadians(this.getYAngle()));
    }

    public double getFieldXMeters() {
        return this.hDiff * Math.cos(Math.toRadians(this.getXAngle()))
                / Math.tan(Math.toRadians(this.getYAngle()));
    }

    public void setPipeLine(double id) {
        pipeLine.setNumber(id);
    }

    public void setTargetHeightDiff(double hDiff) {
        this.hDiff = hDiff;
    }

    public void forceLEDsOff(boolean off) {
        LEDs.setNumber(off ? LimelightConstants.LEDsForceOff : LimelightConstants.LEDsByPipeline);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("xLength", this.getFieldXMeters());
        SmartDashboard.putNumber("yLength", this.getFieldYMeters());
    }
}
