package frc.robot.regressions;

import frc.lib.team254.math.PolynomialRegression;

public class SpeakerShotRegression {
    public static double[][] kWristManualAngle = {
            /* TEMPLATE REGRESSION */
            // @x --> distance from target (in meters)
            // @y --> wrist angle (in degrees)
            { 1.17, 125 },
            { 2.27, 138 },
            {2.61, 139},
            {1.81, 134},
            {2.7, 139.5},
            {3.13, 144},
            //     {3.38, 148}, // 4000, 5000
        //     {4.1, 156}, // 5000, 6000

    };

//     public static double[][] kFlywheelManualRPM = {
//             /* TEMPLATE REGRESSION */
//             // @x --> distance from target (in meters)
//             // @y --> shooter velocity (in rpm)
//             { 0, 0 },

//     };

//     public static PolynomialRegression flywheelRegression = new PolynomialRegression(SpeakerShotRegression.kFlywheelManualRPM,1);

    public static PolynomialRegression wristRegression = new PolynomialRegression(SpeakerShotRegression.kWristManualAngle, 1);



    public static double wristLinearRegression(double meters) {
        return (9.09599 * meters) + 115.829;
    }
}
