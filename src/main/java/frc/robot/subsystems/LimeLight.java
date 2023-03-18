// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
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

    public double getFieldTY() {
        return new Translation2d(getXAngle(), getYAngle()).rotateBy(LimelightConstants.limelightAxisToField).getY();
    }

    public double getFieldTX() {
        return new Translation2d(getXAngle(), getYAngle()).rotateBy(LimelightConstants.limelightAxisToField).getX();
    }

    public double getFieldXArbMeters() {
        return 1 / Math.tan(Math.toRadians(this.getYAngle())); // tan = height/length -> length = height/tan, height in
                                                               // P;
    }

    public double getFieldYArbMeters() {
        return Math.tan(Math.toRadians(this.getXAngle())) * this.getFieldXArbMeters();// tan = offset/length -> offet =
                                                                                      // tan*length;
    }

    public void setPipeLine(double id) {
        pipeLine.setNumber(id);
    }

    public void forceLEDsOff(boolean off) {
        LEDs.setNumber(off ? LimelightConstants.LEDsForceOff : LimelightConstants.LEDsByPipeline);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("xLength", this.getFieldXArbMeters());
        SmartDashboard.putNumber("yLength", this.getFieldYArbMeters());
    }
}
