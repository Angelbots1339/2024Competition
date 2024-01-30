package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PoseEstimation;
import frc.lib.util.SwerveSkewMath;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.logging.loggedObjects.LoggedPigeon2;
import frc.lib.util.logging.loggedObjects.LoggedSweveModules;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LoggingConstants.SwerveLogging;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private LoggedSubsystem logger;
    private LoggedField field;
    private LoggedField autoField;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    // private final SwerveRequest.FieldCentric driveRequestFieldOriented = new
    // SwerveRequest.FieldCentric()
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final SwerveRequest.RobotCentric driveRequestRobotOriented = new
    // SwerveRequest.RobotCentric()
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configPathPlanner();

        CommandScheduler.getInstance().registerSubsystem(this);
        registerTelemetry((SwerveDriveState pose) -> PoseEstimation.updateEstimatedPose(pose, m_modulePositions, this));
        PathPlannerLogging.setLogTargetPoseCallback(PoseEstimation::updateTargetAutoPose);

        if (Utils.isSimulation()) {
            startSimThread();
        }
        initializeLogging();
        PoseEstimation.initNonVisionPoseEstimator(Rotation2d.fromDegrees(this.m_pigeon2.getYaw().getValue()),
                m_kinematics,
                this.m_modulePositions);

        // m_pigeon2.setYaw(0, Constants.kConfigTimeoutSeconds);
    }

    /**
     * 
     * @param translationX  Meters/second
     * @param translationY  Meters/second
     * @param rotation      Rad/second
     * @param fieldOriented Use field oriented drive?
     * @param skewReduction Use Skew Reduction? // TODO Currently doesn't work :(
     * @return
     */
    public Command drive(Supplier<Double> translationX, Supplier<Double> translationY, Supplier<Double> rotation,
            Supplier<Boolean> fieldOriented, Supplier<Boolean> skewReduction) {

        return run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(translationX.get(), translationY.get(), rotation.get());

            if (skewReduction.get()) {
                SwerveSkewMath.reduceSkewFromLogTwist2d(speeds);
            }

            SwerveRequest req;

            if (fieldOriented.get()) {
                req = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(translationX.get()) // Drive forward with negative Y (forward)
                        .withVelocityY(translationY.get()) // Drive left with negative X (left)
                        .withRotationalRate(rotation.get());

            } else {
                req = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(translationX.get()) // Drive forward with negative Y (forward)
                        .withVelocityY(translationY.get()) // Drive left with negative X (left)
                        .withRotationalRate(rotation.get());
            }

            this.setControl(req);
            // System.out.println(translationX.get());
        });
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

        if (Robot.isReal()) {

            if (LimelightHelpers.getFiducialID(VisionConstants.limelight1Name) < 0) {
                return;
            }

            double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelight1Name)
                    .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
            double xyStdDev2 = MathUtil.clamp(0.002 * Math.pow(2.2, tagDistance), 0, 1);

            Pose2d poseFromVision = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.limelight1Name);
            double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                    - (LimelightHelpers.getLatency_Capture(VisionConstants.limelight1Name)
                            + LimelightHelpers.getLatency_Pipeline(VisionConstants.limelight1Name)) / 1000;

            addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
        }

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

    public SwerveModulePosition[] getModulePositions() {
        return m_modulePositions;
    }

    private String command = "None";

    public void initializeLogging() {

        logger = new LoggedSubsystem("Swerve");
        field = new LoggedField("PoseEstimator", logger, SwerveLogging.PoseEstimator, true);

        logger.add(field);
        field.addPose2d("PoseEstimation", () -> this.getState().Pose, true);

        logger.add(new LoggedSweveModules(null, logger, this,
                SwerveLogging.Modules));

        logger.add(new LoggedPigeon2("Gyro", logger, this.m_pigeon2,
                SwerveLogging.Gyro));

        logger.addDouble("Heading", () -> this.getState().Pose.getRotation().getDegrees(), SwerveLogging.Pose);
        logger.addDouble("x velocity", () -> PoseEstimation.getEstimatedVelocity().getX(), SwerveLogging.Pose);
        logger.addDouble("y velocity", () -> PoseEstimation.getEstimatedVelocity().getX(), SwerveLogging.Pose);

        logger.addString("Command", () -> {
            Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {
                command = c.getName();
            });
            return command;
        }, SwerveLogging.Main);

        logger.addDouble("xAutoError", () -> PoseEstimation.getAutoTargetPoseError().getX(), SwerveLogging.Auto);
        logger.addDouble("yAutoError", () -> PoseEstimation.getAutoTargetPoseError().getY(), SwerveLogging.Auto);
        logger.addDouble("thetaAutoError", () -> PoseEstimation.getAutoTargetPoseError().getRotation().getDegrees(),
                SwerveLogging.Auto);

    }

}