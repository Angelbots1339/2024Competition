package frc.lib.util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.Swerve;

public class PoseEstimation {

    private static Pose2d estimatedPose = new Pose2d();
    private static Pose2d estimatedVelocity = new Pose2d();
    private static Pose2d targetAutoPose = new Pose2d();

    private static double lastTime = 0;
    private static Pose2d lastPose = new Pose2d();

    private static SwerveDrivePoseEstimator poseEstimatorNonVision;

    public static void initNonVisionPoseEstimator(Rotation2d rot, SwerveDriveKinematics kinematics, SwerveModulePosition[] positions) {
        poseEstimatorNonVision = new SwerveDrivePoseEstimator(kinematics, rot, positions, estimatedPose);
    }

    /**
     * Don't call this. It's registered to the Swerve telemetry in the Swerve
     * subsystem
     * 
     * @param state
     */
    public static void updateEstimatedPose(SwerveDriveState state, SwerveModulePosition[] modulePositions, Swerve swerve) {

        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Pose2d distanceDiff = new Pose2d().transformBy(state.Pose.minus(lastPose));
        lastPose = state.Pose;
        Pose2d velocities = distanceDiff.div(diffTime);

        estimatedVelocity = velocities;
        estimatedPose = state.Pose;

        
        // poseEstimatorNonVision.update(Rotation2d.fromDegrees(swerve.getPigeon2().getYaw().getValue()), swerve.getModulePositions());
        // poseEstimatorNonVision.update(Rotation2d.fromDegrees(0), modulePositions);

    }

    /**
     * Get the estimated pose from the Swerve Subsystem
     * 
     * @return Current robot pose
     */
    public static Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    /**
     * Get velocity based on the estimated pose
     * 
     * @return Current robot velocity
     */
    public static Pose2d getEstimatedVelocity() {
        return estimatedVelocity;
    }

    /**
     * Used for shooting while moving to make a virtual target offset based on the
     * robot's current velocity
     * 
     * @return the translated target position
     */
    public static Translation2d calculateVirtualSpeakerOffset(Translation2d targetPosition) {

        // TODO Do math here

        

        return targetPosition;
    }

    /**
     * DO NOT CALL
     * 
     * This should be called periodically by Pathplanner's
     * PathPlannerLogging.setLogTargetPoseCallback()
     */
    public static void updateTargetAutoPose(Pose2d pose) {

        targetAutoPose = pose;
    }

    /**
     * Get the pose the robot is trying to drive to in auto. It's straight from
     * PathPlanner
     * 
     * @return Target robot pose
     */
    public static Pose2d getTargetAutoPose() {

        return targetAutoPose;
    }

    /**
     * See how far off the robot is from Pathplanner's target pose
     * 
     * @return Target pose error
     */
    public static Pose2d getAutoTargetPoseError() {

        return new Pose2d().transformBy(targetAutoPose.minus(estimatedPose));
    }

}