package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class SwerveSkewMath {
    
    /*-----Possible Solutions for Swerve Drive Skew------ */

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5?u=ethan1}
     */
    public static ChassisSpeeds reduceSkewFromChassisSpeeds254(ChassisSpeeds chassisSpeeds, double LooperDT) {
        frc.lib.team254.geometry.Pose2d robot_pose_vel = new frc.lib.team254.geometry.Pose2d(
                chassisSpeeds.vxMetersPerSecond * LooperDT,
                chassisSpeeds.vyMetersPerSecond * LooperDT,
                frc.lib.team254.geometry.Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * LooperDT));
        frc.lib.team254.geometry.Twist2d twist_vel = frc.lib.team254.geometry.Pose2d.log(robot_pose_vel);
        return new ChassisSpeeds(
                twist_vel.dx / LooperDT, twist_vel.dy / LooperDT, twist_vel.dtheta / LooperDT);
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
     */

    private static double previousDriveTime;

    public static ChassisSpeeds reduceSkewFromLogTwist2d(ChassisSpeeds chassisSpeeds) {
        double timerDt = (Timer.getFPGATimestamp() - previousDriveTime);
        Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * timerDt,
                chassisSpeeds.vyMetersPerSecond * timerDt,
                Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * timerDt));
        Twist2d twist_vel = new Pose2d(0, 0, new Rotation2d(0)).log(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / timerDt, twist_vel.dy / timerDt, twist_vel.dtheta / timerDt);
        previousDriveTime = Timer.getFPGATimestamp();
        return updated_chassis_speeds;
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
     */
    public static ChassisSpeeds reduceSkewFromChassisSpeedsFudgeFactor(ChassisSpeeds chassisSpeeds, double FudgeFactorKp) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);
        double fudgeFactor = chassisSpeeds.omegaRadiansPerSecond / linearVelocity * FudgeFactorKp;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthY = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthY;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         Takes the unit orthogonal vector and scales it by a constant times
     *         the angular velocity
     */
    public static ChassisSpeeds reduceSkewFromChassisSpeedsSimpleFudgeFactor(ChassisSpeeds chassisSpeeds, double FudgeFactorSimpleKp) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);

        double fudgeFactor = Math.signum(chassisSpeeds.omegaRadiansPerSecond) * FudgeFactorSimpleKp;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthY = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthY;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }
}
