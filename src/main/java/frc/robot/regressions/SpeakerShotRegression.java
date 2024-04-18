package frc.robot.regressions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team254.math.InterpolatingDouble;
import frc.lib.team254.math.InterpolatingTreeMap;
import frc.lib.team254.math.PolynomialRegression;
import frc.robot.Constants.ScoringConstants;

public class SpeakerShotRegression {
    public static double[][] kWristManualAngle = {
            // x --> distance from target (in meters)
            // y --> wrist angle (in degrees)
            { 1.375, 123 }, // 127
            { 1.581, 128 }, // 131
            { 1.774, 132 }, // 134
            { 1.975, 135 }, // 137.5
            { 2.265, 139 }, // 141.5
            { 2.464, 142.5 }, // 143
            { 2.647, 144.5 }, // 145  Above is Dresden changes
            { 2.835, 146.8 }, // 147 Practice Field Change
            { 3.159, 148.3 }, // 148.5  Practice Field Change
            { 3.35, 149.8 },  // 150 Practice Field Change
            { 3.516, 150.5 }, 
            { 3.755, 151.5 }, 
            { 3.963, 152.6 }, 
            { 4.147, 153.2 }, 
            { 4.352, 153.5 }, 
            { 4.567, 154.3 }, 
            { 4.76, 154.8 },
            { 4.91, 155 },
            { 5.35, 155.5 }, 
            { 5.5, 156},
            { 5.75, 156.3 }, // 5.6 barely high enough
            { 6, 156.4 },
            { 6.25, 156.6}, // lower means higher number
            
            { 6.5, 156.95 },
            { 6.75, 157.21},
            { 7, 157.45 },

    };


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
        ScoringConstants.wristRegressionMinClamp, ScoringConstants.wristRegressionMaxClamp))
        .plus(Rotation2d.fromDegrees(-33)); // I hate my life
        // higher negative number => shoot higher
        // lower negative number => shoot lower
        // -33.2 was shooting high from distance in Match 26
        // 
        
        //     return Rotation2d.fromDegrees(MathUtil.clamp(wristExpoRegression(targetDistance),
        // ScoringConstants.wristRegressionMinClamp, ScoringConstants.wristRegressionMaxClamp));

        //     return Rotation2d.fromDegrees(MathUtil.clamp(wristRegression.predict(targetDistance),
        // ScoringConstants.wristRegressionMinClamp, ScoringConstants.wristRegressionMaxClamp));
    }
}
