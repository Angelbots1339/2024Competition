package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PoseEstimation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        CommandScheduler.getInstance().registerSubsystem(this);
        registerTelemetry((SwerveDriveState pose) -> PoseEstimation.updateEstimatedPose(pose.Pose));
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        CommandScheduler.getInstance().registerSubsystem(this);
        registerTelemetry((SwerveDriveState pose) -> PoseEstimation.updateEstimatedPose(pose.Pose));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void updateVision() {

        if (LimelightHelpers.getFiducialID(VisionConstants.kLimelightName) < 0) {
            return;
        }

        double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.kLimelightName)
                .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
        double xyStdDev2 = MathUtil.clamp(0.002 * Math.pow(2.2, tagDistance), 0, 1);

        Pose2d poseFromVision = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.kLimelightName);
        double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                - (LimelightHelpers.getLatency_Capture(VisionConstants.kLimelightName)
                        + LimelightHelpers.getLatency_Pipeline(VisionConstants.kLimelightName)) / 1000;

        addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(xyStdDev2, xyStdDev2, 0));

    }

    public void periodic() {

        updateVision();

    }

    public void configPathPlanner() {

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Robot pose supplier
                this::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                AutoConstants.autoConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        this.setControl(autoRequest.withSpeeds(speeds));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

}