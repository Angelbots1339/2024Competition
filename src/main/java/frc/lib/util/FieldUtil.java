package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {

    public static final Translation2d BlueSpeakerPosition = new Translation2d(8.27 - 8.308467, 4.105 + 1.442593);
    public static final Translation2d RedSpeakerPosition = new Translation2d(8.27 + 8.308467, 4.105 + 1.442593);

    public static final Translation2d BlueAmpPosition = new Translation2d(8.27 - 6.429375, 4.105 - 4.098925);
    public static final Translation2d RedAmpPosition = new Translation2d(8.27 + 6.429883, 4.105 + 4.098925);

    /**
     * Get the position of whichever alliance you are on
     * 
     * @return the speaker position
     */
    public static Translation2d getAllianceSpeakerPosition() {

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue ? BlueSpeakerPosition : RedSpeakerPosition;
        }

        return BlueSpeakerPosition;
    }

    /**
     * Get the position of whichever alliance you are on
     * 
     * @return the speaker position
     */
    public static Translation2d getAllianceAmpPosition() {

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue ? BlueAmpPosition : RedAmpPosition;
        }

        return BlueAmpPosition;
    }

    public static boolean isAllianceBlue() {
        boolean isAllianceBlue = true;

        if (DriverStation.getAlliance().isPresent()) {
            isAllianceBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        }

        return isAllianceBlue;
    }

}
