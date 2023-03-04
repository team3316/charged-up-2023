// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.LimelightConstants;

public class LimeLight extends SubsystemBase {
    NetworkTable limeLightTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pipeLine;
    private NetworkTableEntry LEDs;

    public class BotPose {
        private Pose2d pose;
        private double timestamp;

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public BotPose(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

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

    public void setPipeLine(double id) {
        pipeLine.setNumber(id);
    }

    public void forceLEDsOff(boolean off) {
        LEDs.setNumber(off ? LimelightConstants.LEDsForceOff : LimelightConstants.LEDsByPipeline);
    }
    
    /*
     * Can return null
     */
    public BotPose getBotPose() {
        if (!hasTarget())
            return null;

        String entryName = DriverStation.getAlliance() == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";

        double[] poseComponents = limeLightTable.getEntry(entryName).getDoubleArray(new double[6]);
        if (poseComponents.length == 0)
            return null;

        return new BotPose(new Pose2d(
                poseComponents[0],
                poseComponents[1],
                new Rotation2d(Math.toRadians(poseComponents[5]))),
                Timer.getFPGATimestamp() - (poseComponents[6] / 1000.0));

    }

    public PathPlannerTrajectory getCollectionTrajectory() {
        Pose2d botPose = getBotPose().getPose();
        if (botPose == null) {
            return new PathPlannerTrajectory();
        }

        // heading straight to the collection using vector subtraction
        Rotation2d startingHeading = LimelightConstants.collectionTranslation
                .minus(botPose.getTranslation()).getAngle();

        return PathPlanner.generatePath(
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathPoint(botPose.getTranslation(), startingHeading, botPose.getRotation()),
                LimelightConstants.collectionPathPoint);
    }
}
