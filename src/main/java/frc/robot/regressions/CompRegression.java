package frc.robot.regressions;

import frc.lib.util.math.PolynomialRegression;

public class CompRegression {
    public static double[][] kWristManualAngle = {
            /* TEMPLATE REGRESSION */
            // @x --> distance from target (in meters)
            // @y --> hood angle (in degrees)
            { 0, 0 },

    };

    public static double[][] kFlywheelManualRPM = {
            /* TEMPLATE REGRESSION */
            // @x --> distance from target (in meters)
            // @y --> shooter velocity (in rpm)
            { 0, 0 },

    };

    public static PolynomialRegression flywheelRegression = new PolynomialRegression(CompRegression.kFlywheelManualRPM,1);

    public static PolynomialRegression wristRegression = new PolynomialRegression(CompRegression.kWristManualAngle, 1);

}
