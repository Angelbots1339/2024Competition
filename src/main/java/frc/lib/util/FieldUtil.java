package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {

    public static final Translation2d BlueSpeakerPose = new Translation2d(8.27 - 7.24310, 4.105 - 1.26019);
    public static final Translation2d RedSpeakerPose = new Translation2d(8.27 + 8.308467, 4.105 + 1.442593);

    /**
     * Get the position of whichever alliance you are on
     * 
     * @return the speaker position
     */
    public static Translation2d getAllianceSpeakerPosition() {

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return RedSpeakerPose;
            } else {
                return BlueSpeakerPose;
            }
        }

        return null;
    }

}
