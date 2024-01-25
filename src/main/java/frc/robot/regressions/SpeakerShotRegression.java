package frc.robot.regressions;

import frc.lib.team254.math.PolynomialRegression;

public class SpeakerShotRegression {
    public static double[][] kWristManualAngle = {
            /* TEMPLATE REGRESSION */
            // @x --> distance from target (in meters)
            // @y --> wrist angle (in degrees)
            { 0, 0 },

    };

    public static double[][] kFlywheelManualRPM = {
            /* TEMPLATE REGRESSION */
            // @x --> distance from target (in meters)
            // @y --> shooter velocity (in rpm)
            { 0, 0 },

    };

    public static PolynomialRegression flywheelRegression = new PolynomialRegression(SpeakerShotRegression.kFlywheelManualRPM,1);

    public static PolynomialRegression wristRegression = new PolynomialRegression(SpeakerShotRegression.kWristManualAngle, 1);

}
