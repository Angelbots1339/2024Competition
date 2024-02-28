package frc.robot.regressions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team254.math.InterpolatingDouble;
import frc.lib.team254.math.InterpolatingTreeMap;
import frc.lib.team254.math.PolynomialRegression;
import frc.robot.Constants.ScoringConstants;

public class SpeakerShotRegression {
    public static double[][] kWristManualAngle = {
            /* TEMPLATE REGRESSION */
            // @x --> distance from target (in meters)
            // @y --> wrist angle (in degrees)
            { 1.17, 120 },
            { 2.27, 138 },
            { 2.61, 139 },
            { 1.81, 134 },
            { 2.7, 139.5 },
            { 3.13, 144 },
            { 3.38, 148 }, // 4000, 5000
            { 3.41, 148.5 }, // 5000, 6000
            { 3.825, 150 }, // 5000, 6000
            { 4.045, 151 }, // 5000, 6000
            { 4.375, 152 }, // 5000, 6000 ?
            { 4.61, 152.25 }, // 5000, 6000 ?
            { 5.3, 153.75 }, // 5000, 6000 ?

    };

    // public static double[][] kFlywheelManualRPM = {
    // /* TEMPLATE REGRESSION */
    // // @x --> distance from target (in meters)
    // // @y --> shooter velocity (in rpm)
    // { 0, 0 },

    // };

    // public static PolynomialRegression flywheelRegression = new
    // PolynomialRegression(SpeakerShotRegression.kFlywheelManualRPM,1);

    public static PolynomialRegression wristRegression = new PolynomialRegression(
            SpeakerShotRegression.kWristManualAngle, 1);

    public static double wristExpoRegression(double meters) {
        return Math.pow(122.462 * meters, 0.145);
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> wristInterpolation = new InterpolatingTreeMap<>();
    static {
        for(double[] setpoint : kWristManualAngle){
            wristInterpolation.put(new InterpolatingDouble(setpoint[0]), new InterpolatingDouble(setpoint[1]));
        }
    }

    public static Rotation2d calculateWristAngle(double targetDistance) {

            return Rotation2d.fromDegrees(MathUtil.clamp(wristInterpolation.getInterpolated(new InterpolatingDouble(targetDistance)).value,
        ScoringConstants.wristRegressionMinClamp, ScoringConstants.wristRegressionMaxClamp));
        
        //     return Rotation2d.fromDegrees(MathUtil.clamp(wristExpoRegression(targetDistance),
        // ScoringConstants.wristRegressionMinClamp, ScoringConstants.wristRegressionMaxClamp));

        //     return Rotation2d.fromDegrees(MathUtil.clamp(wristRegression.predict(targetDistance),
        // ScoringConstants.wristRegressionMinClamp, ScoringConstants.wristRegressionMaxClamp));
    }
}
