package frc.robot.subsystems.drivetrain;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.BotPose;

public class PoseEstimator extends SubsystemBase {

    private final LimeLight limeLight;
    private final Drivetrain drivetrain;

    private double lastVisionTimestamp = -1;

    // TODO: these values are only approximation from PP
    private static final Translation2d collectionTranslation = new Translation2d(14.0, 7.5);
    private static final Rotation2d collectionHolonomicRotation = Rotation2d.fromDegrees(90);
    private static final Rotation2d collectionHeading = Rotation2d.fromDegrees(90);
    private static final PathPoint collectionPathPoint = new PathPoint(
            collectionTranslation,
            collectionHeading,
            collectionHolonomicRotation);

    // maximum allowed deviation in meters between current pose and vision pose in
    // order to accept the vision reading as valid.
    private static final double visionMeasurementRejectionThreshold = 2.0;

    private final double xTolerance = 0.2; // in meters
    private final double yTolerance = 0.1; // in meters
    private final double tTolerance = 10; // in degrees

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();

    public PoseEstimator(Drivetrain drivetrain, LimeLight limeLight) {
        this.limeLight = limeLight;
        this.drivetrain = drivetrain;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");

        poseEstimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.kinematics,
                drivetrain.getRotation2d(),
                drivetrain.getSwerveModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.addBoolean("Vision Synced", this::isVisionSynced).withPosition(0, 1).withSize(2, 1);
        tab.addBoolean("X at SSS", this::xAtSSS).withPosition(0, 2).withSize(1, 1);
        tab.addNumber("X error", this::xError).withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of(
                        "min", collectionTranslation.getX() - (xTolerance * 5),
                        "max", collectionTranslation.getX() + (xTolerance * 5)))
                .withPosition(1, 2)
                .withSize(1, 1);
        tab.addBoolean("Y at SSS", this::yAtSSS).withPosition(0, 3).withSize(1, 1);
        tab.addNumber("Y error", this::yError).withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of(
                        "min", collectionTranslation.getY() - (xTolerance * 5),
                        "max", collectionTranslation.getY() + (xTolerance * 5)))
                .withPosition(1, 3)
                .withSize(1, 1);
        tab.addBoolean("T at SSS", this::tAtSSS).withPosition(0, 4).withSize(1, 1);
        tab.addNumber("T error", this::tError).withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of(
                        "min", collectionHolonomicRotation.getDegrees() - (tTolerance * 5),
                        "max", collectionHolonomicRotation.getDegrees() + (tTolerance * 5)))
                .withPosition(1, 4)
                .withSize(1, 1);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }

    private void addValidVisionMeasurement(BotPose robotPose) {
        if (robotPose == null) {
            return;
        }

        if (getCurrentPose().minus(robotPose.getPose()).getTranslation()
                .getNorm() > visionMeasurementRejectionThreshold) {
            return;
        }

        lastVisionTimestamp = robotPose.getTimestamp();
        poseEstimator.addVisionMeasurement(robotPose.getPose(), robotPose.getTimestamp());
    }

    @Override
    public void periodic() {
        // Update pose estimator with the best visible target
        addValidVisionMeasurement(limeLight.getBotPose());

        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
                drivetrain.getRotation2d(),
                drivetrain.getSwerveModulePositions());

        field2d.setRobotPose(getCurrentPose());
    }

    private boolean isVisionSynced() {
        return (Timer.getFPGATimestamp() - lastVisionTimestamp) < 5;
    }

    private double xError() {
        return getCurrentPose().getTranslation().minus(collectionTranslation).getX();
    }

    private double yError() {
        return getCurrentPose().getTranslation().minus(collectionTranslation).getY();
    }

    private double tError() {
        return getCurrentPose().getRotation().minus(collectionHolonomicRotation).getDegrees();
    }

    private boolean xAtSSS() {
        return Math.abs(xError()) < xTolerance;
    }

    private boolean yAtSSS() {
        return Math.abs(yError()) < yTolerance;
    }

    private boolean tAtSSS() {
        return Math.abs(tError()) < tTolerance;
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * 
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                drivetrain.getRotation2d(),
                drivetrain.getSwerveModulePositions(),
                newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    public PathPlannerTrajectory getCollectionTrajectory() {
        var pose = getCurrentPose();
        var speeds = drivetrain.getChassisSpeeds();

        var velocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        var heading = Rotation2d.fromRadians(Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        return PathPlanner.generatePath(
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathPoint(pose.getTranslation(), heading.plus(pose.getRotation()), pose.getRotation(), velocity),
                collectionPathPoint);
    }

}