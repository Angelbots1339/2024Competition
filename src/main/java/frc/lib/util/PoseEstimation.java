package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseEstimation {
    
    public static Pose2d estimatedPose = new Pose2d();

    /**
     * Don't call this. It's registered to the Swerve telemetry in the Swerve subsystem
     * 
     * @param pose
     */
    public static void updateEstimatedPose(Pose2d pose) {
        estimatedPose = pose;
    }


    /**
     * Used for shooting while moving to make a virtual target offset
     * 
     * @return the translated target position
     */
    public static Translation2d calculateSpeakerOffset() {

        // TODO Do math here

        return FieldUtil.getAllianceSpeakerPosition();
    }


    



}
