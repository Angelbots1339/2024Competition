package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;
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

    private PIDController angularDrivePID = new PIDController(DriverConstants.angularDriveKP,
            DriverConstants.angularDriveKI, DriverConstants.angularDriveKD);

    private PIDController pidToPoseXController = new PIDController(DriverConstants.pidToPoseKP, 0,
            DriverConstants.pidToPoseKD);
    private PIDController pidToPoseYController = new PIDController(DriverConstants.pidToPoseKP, 0,
            DriverConstants.pidToPoseKD);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private boolean overrideAutoRotation = false;
    private Supplier<Rotation2d> autoRotation = () -> new Rotation2d();

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

        // registerTelemetry((SwerveDriveState pose) ->
        // PoseEstimation.updateEstimatedPose(pose, m_modulePositions, this));

        PathPlannerLogging.setLogTargetPoseCallback(PoseEstimation::updateTargetAutoPose);

        initializeLogging();
        PoseEstimation.initNonVisionPoseEstimator(Rotation2d.fromDegrees(this.m_pigeon2.getYaw().getValue()),
                m_kinematics,
                this.m_modulePositions);

        angularDrivePID.setTolerance(DriverConstants.angularDriveTolerance);
        angularDrivePID.enableContinuousInput(0, 360);
        pidToPoseXController.setTolerance(DriverConstants.pidToPoseTolerance);
        pidToPoseXController.setTolerance(DriverConstants.pidToPoseTolerance);

        if (Utils.isSimulation()) {
            startSimThread();
        }
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

                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw());

                req = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                        .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
                        .withRotationalRate(fieldRelativeSpeeds.omegaRadiansPerSecond);

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

    /**
     * 
     * @param translationX  Meters/second
     * @param translationY  Meters/second
     * @param rotation      Rad/second
     * @param fieldOriented Use field oriented drive?
     * @param skewReduction Use Skew Reduction? // TODO Currently doesn't work :(
     * @return
     */
    public Command angularDrive(Supplier<Double> translationX, Supplier<Double> translationY,
            Supplier<Rotation2d> desiredRotation,
            Supplier<Boolean> fieldOriented, Supplier<Boolean> skewReduction) {

        return run(() -> {

            // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            // rot = rot.plus(Rotation2d.fromDegrees(180));
            // };

            ChassisSpeeds speeds = angularPIDCalc(translationX, translationY, desiredRotation);

            if (skewReduction.get()) {
                speeds = SwerveSkewMath.reduceSkewFromLogTwist2d(speeds);
            }

            SwerveRequest req;

            if (fieldOriented.get()) {
                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw());

                req = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                        .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
                        .withRotationalRate(fieldRelativeSpeeds.omegaRadiansPerSecond);

            } else {
                req = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(translationX.get())
                        .withVelocityY(translationY.get())
                        .withRotationalRate(speeds.omegaRadiansPerSecond);
            }

            this.setControl(req);
        });
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
    public SwerveRequest angularDriveRequest(Supplier<Double> translationX, Supplier<Double> translationY,
            Supplier<Rotation2d> desiredRotation, Supplier<Boolean> skewReduction) {

        

            // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            // rot = rot.plus(Rotation2d.fromDegrees(180));
            // };

            ChassisSpeeds speeds = angularPIDCalc(translationX, translationY, desiredRotation);

            if (skewReduction.get()) {
                speeds = SwerveSkewMath.reduceSkewFromLogTwist2d(speeds);
            }

            SwerveRequest req;

                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw());

                req = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                        .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
                        .withRotationalRate(fieldRelativeSpeeds.omegaRadiansPerSecond);

            return req;
    }

    /**
     * Builds a ChassisSpeeds with the given translation and the output of the
     * anularPID for rotation
     * 
     * @param translationX
     * @param translationY
     * @param desiredRotation
     * @return New ChassisSpeeds
     */
    private ChassisSpeeds angularPIDCalc(Supplier<Double> translationX, Supplier<Double> translationY,
            Supplier<Rotation2d> desiredRotation) {
        double pid = angularDrivePID.calculate(this.getAdjustedYaw().getDegrees(), desiredRotation.get().getDegrees());

        ChassisSpeeds speeds = new ChassisSpeeds(translationX.get(), translationY.get(),
                angularDrivePID.atSetpoint() ? 0 : pid + (DriverConstants.angularDriveKS * Math.signum(pid)));

                
        return speeds;
    }

    public boolean isAtAngularDriveSetpoint() {
        return angularDrivePID.atSetpoint();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void pidToPose(Pose2d target) {

        double x = -MathUtil.clamp(pidToPoseXController.calculate(PoseEstimation.getEstimatedPose().getTranslation().getX(),
                target.getX())
                + Math.signum(pidToPoseXController.getPositionError()) * Math.abs(DriverConstants.pidToPoseKS), -DriverConstants.pidToPoseMaxSpeed, DriverConstants.pidToPoseMaxSpeed);
        double y = -MathUtil.clamp(pidToPoseYController.calculate(PoseEstimation.getEstimatedPose().getTranslation().getY(),
                target.getY())
                + Math.signum(pidToPoseXController.getPositionError()) * Math.abs(DriverConstants.pidToPoseKS), -DriverConstants.pidToPoseMaxSpeed, DriverConstants.pidToPoseMaxSpeed);

        // SmartDashboard.putNumber("PidXError", pidToPoseXController.getPositionError());
        // SmartDashboard.putNumber("PidYError", pidToPoseYController.getPositionError());

        this.setControl(angularDriveRequest(() -> pidToPoseXController.atSetpoint() ? 0 : x, () -> pidToPoseYController.atSetpoint() ? 0 : y, () -> target.getRotation(), () -> false));
    }

    public boolean isAtPose() {
        return pidToPoseXController.atSetpoint() && pidToPoseYController.atSetpoint() && isAtAngularDriveSetpoint();
    }

    /**
     * Hijacks the robot's rotation but keeps PathPlanner translation. Used for
     * shooting while following the path
     * 
     * @param speeds
     */
    public void setChassisSpeedsAuto(ChassisSpeeds speeds) {
        if (overrideAutoRotation) {

            ChassisSpeeds newSpeeds = angularPIDCalc(() -> speeds.vxMetersPerSecond, () -> speeds.vyMetersPerSecond,
                    autoRotation);

            this.setControl(autoRequest.withSpeeds(newSpeeds));
        } else {
            this.setControl(autoRequest.withSpeeds(speeds));

        }

    }

    public void setAutoOverrideRotation(boolean override) {
        overrideAutoRotation = override;
    }

    public void setAutoOverrideRotation(boolean override, Supplier<Rotation2d> rotation) {
        overrideAutoRotation = override;
        autoRotation = rotation;
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public SwerveModulePosition[] getModulePositions() {
        return m_modulePositions;
    }

    public Rotation2d getGyroYaw() {
        double rawYaw = m_pigeon2.getYaw().getValue();
        double yawWithRollover = rawYaw > 0 ? rawYaw % 360 : 360 - Math.abs(rawYaw % 360);
        
        return Rotation2d.fromDegrees(yawWithRollover);
    }
    public Rotation2d getAdjustedYaw() {
        return DriverStation.getAlliance().get() == Alliance.Blue ? getGyroYaw()
                : getGyroYaw().plus(new Rotation2d(Math.PI));
    }


    public void setGyroYaw(Rotation2d yaw) {
        m_pigeon2.setYaw(yaw.getDegrees(), Constants.kConfigTimeoutSeconds);
    }
    public void zeroGyro() {
        setGyroYaw(Rotation2d.fromDegrees(0));
    }
    public void zeroGyro(Rotation2d rot) {
        setGyroYaw(Rotation2d.fromDegrees(rot.getDegrees()));
    }


    public void updateVision() {
        if (Robot.isReal()) {

            if (LimelightHelpers.getFiducialID(VisionConstants.limelightLeftName) >= 0) {
                double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelightLeftName)
                        .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
                double xyStdDev2 = VisionConstants.calcStdDev(tagDistance);

                Pose2d poseFromVision = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.limelightLeftName);
                double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                        - (LimelightHelpers.getLatency_Capture(VisionConstants.limelightLeftName)
                                + LimelightHelpers.getLatency_Pipeline(VisionConstants.limelightLeftName)) / 1000;

                addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
            }

            if (LimelightHelpers.getFiducialID(VisionConstants.limelightCenterName) >= 0) {
                double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelightCenterName)
                        .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
                double xyStdDev2 = VisionConstants.calcStdDev(tagDistance);

                Pose2d poseFromVision = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.limelightCenterName);
                double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                        - (LimelightHelpers.getLatency_Capture(VisionConstants.limelightCenterName)
                                + LimelightHelpers.getLatency_Pipeline(VisionConstants.limelightCenterName)) / 1000;

                addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
            }

            if (LimelightHelpers.getFiducialID(VisionConstants.limelightRightName) >= 0) {
                double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelightRightName)
                        .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
                double xyStdDev2 = VisionConstants.calcStdDev(tagDistance);

                Pose2d poseFromVision = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.limelightRightName);
                double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                        - (LimelightHelpers.getLatency_Capture(VisionConstants.limelightRightName)
                                + LimelightHelpers.getLatency_Pipeline(VisionConstants.limelightRightName)) / 1000;

                addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
            }

        }
    }

    public void periodic() {

        updateVision();
        PoseEstimation.updateEstimatedPose(this.m_odometry.getEstimatedPosition(), m_modulePositions, this);
    }

    public void resetPose(Pose2d pose) {
        this.seedFieldRelative(pose);
        this.zeroGyro(pose.getRotation());
    }

    public void configPathPlanner() {

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeedsAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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

    private String command = "None";

    public void initializeLogging() {

        logger = new LoggedSubsystem("Swerve");
        field = new LoggedField("PoseEstimator", logger, SwerveLogging.Pose, true);

        logger.add(field);
        field.addPose2d("PoseEstimation", () -> PoseEstimation.getEstimatedPose(), true);

        logger.add(new LoggedSweveModules(null, logger, this,
                SwerveLogging.Modules));

        logger.add(new LoggedPigeon2("Gyro", logger, this.m_pigeon2,
                SwerveLogging.Gyro));

        logger.addDouble("Heading", () -> PoseEstimation.getEstimatedPose().getRotation().getDegrees(),
                SwerveLogging.Pose);
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

        logger.addBoolean("isAtAngularDriveSetpoint", () -> isAtAngularDriveSetpoint(), SwerveLogging.PidPose);

    }

}