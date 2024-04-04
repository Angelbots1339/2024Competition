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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PoseEstimation;
import frc.lib.util.SwerveSkewMath;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.logging.loggedObjects.LoggedPigeon2;
import frc.lib.util.logging.loggedObjects.LoggedSweveModules;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
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

    private LoggedSubsystem logger = new LoggedSubsystem("Swerve");
    private LoggedField field;
    private LoggedSweveModules loggedModules;

    private PIDController angularDrivePID = new PIDController(DriverConstants.angularDriveKP,
            DriverConstants.angularDriveKI, DriverConstants.angularDriveKD);

    private PIDController pidToPoseXController = new PIDController(DriverConstants.pidToPoseKP, 0,
            DriverConstants.pidToPoseKD);
    private PIDController pidToPoseYController = new PIDController(DriverConstants.pidToPoseKP, 0,
            DriverConstants.pidToPoseKD);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private boolean overrideAutoRotation = false;
    private Supplier<Rotation2d> autoRotation = () -> new Rotation2d();
    
    private double lastYaw = 0;
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

        registerTelemetry(
                (SwerveDriveState pose) -> PoseEstimation.updateEstimatedPose(this.m_odometry.getEstimatedPosition(),
                        m_modulePositions, this));

        PathPlannerLogging.setLogTargetPoseCallback(PoseEstimation::updateTargetAutoPose);

        initializeLogging();
        // PoseEstimation.initNonVisionPoseEstimator(Rotation2d.fromDegrees(this.m_pigeon2.getYaw().getValue()),
        // m_kinematics,
        // this.m_modulePositions);

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
     * @param skewReduction Use Skew Reduction?
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

                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAdjustedYaw());

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
     * @param skewReduction Use Skew Reduction?
     * @return Drive Command
     */
    public Command angularDrive(Supplier<Double> translationX, Supplier<Double> translationY,
            Supplier<Rotation2d> desiredRotation,
            Supplier<Boolean> fieldOriented, Supplier<Boolean> skewReduction) {

        return run(() -> {

            ChassisSpeeds speeds = angularPIDCalc(translationX, translationY, desiredRotation);

            if (skewReduction.get()) {
                speeds = SwerveSkewMath.reduceSkewFromLogTwist2d(speeds);
            }

            SwerveRequest req;

            if (fieldOriented.get()) {
                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAdjustedYaw());

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
     * Generates and applies the angular drive request. Must call periodically.
     * 
     * @param translationX  Meters/second
     * @param translationY  Meters/second
     * @param rotation      Rad/second
     * @param fieldOriented Use field oriented drive?
     * @param skewReduction Use Skew Reduction?
     * 
     */
    public void angularDriveRequest(Supplier<Double> translationX, Supplier<Double> translationY,
            Supplier<Rotation2d> desiredRotation, Supplier<Boolean> skewReduction) {

        ChassisSpeeds speeds = angularPIDCalc(translationX, translationY, desiredRotation);

        if (skewReduction.get()) {
            speeds = SwerveSkewMath.reduceSkewFromLogTwist2d(speeds);
        }

        SwerveRequest req;

        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAdjustedYaw());

        req = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                        : DriveRequestType.Velocity)
                .withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
                .withRotationalRate(fieldRelativeSpeeds.omegaRadiansPerSecond);

        this.setControl(req);
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
        double pid = angularDrivePID.calculate(getAdjustedYaw().getDegrees(), desiredRotation.get().getDegrees());

        ChassisSpeeds speeds = new ChassisSpeeds(translationX.get(), translationY.get(),
                MathUtil.clamp(
                        angularDrivePID.atSetpoint() ? 0 : pid + (DriverConstants.angularDriveKS * Math.signum(pid)),
                        -SwerveConstants.maxAngularRate, SwerveConstants.maxAngularRate));

        return speeds;
    }

    public boolean isAngularDriveAtSetpoint() {
        return angularDrivePID.atSetpoint();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void pidToPose(Pose2d target) {

        double x = -MathUtil.clamp(
                pidToPoseXController.calculate(PoseEstimation.getEstimatedPose().getTranslation().getX(),
                        target.getX())
                        + Math.signum(pidToPoseXController.getPositionError()) * Math.abs(DriverConstants.pidToPoseKS),
                -DriverConstants.pidToPoseMaxSpeed, DriverConstants.pidToPoseMaxSpeed);
        double y = -MathUtil.clamp(
                pidToPoseYController.calculate(PoseEstimation.getEstimatedPose().getTranslation().getY(),
                        target.getY())
                        + Math.signum(pidToPoseXController.getPositionError()) * Math.abs(DriverConstants.pidToPoseKS),
                -DriverConstants.pidToPoseMaxSpeed, DriverConstants.pidToPoseMaxSpeed);

        // SmartDashboard.putNumber("PidXError",
        // pidToPoseXController.getPositionError());
        // SmartDashboard.putNumber("PidYError",
        // pidToPoseYController.getPositionError());

        angularDriveRequest(() -> pidToPoseXController.atSetpoint() ? 0 : x,
                () -> pidToPoseYController.atSetpoint() ? 0 : y, () -> target.getRotation(), () -> false);
    }

    public boolean isAtPose() {
        return pidToPoseXController.atSetpoint() && pidToPoseYController.atSetpoint() && isAngularDriveAtSetpoint();
    }

    /**
     * Hijacks the robot's rotation but keeps PathPlanner translation. Used for
     * shooting while following the path
     * 
     * @param speeds
     */
    public void setChassisSpeedsAuto(ChassisSpeeds speeds) {
        if (overrideAutoRotation) {
            PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(autoRotation.get()));
        } else {
            PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
        }
        this.setControl(autoRequest.withSpeeds(speeds));

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

        double rawYaw = m_pigeon2.getYaw().getValue() + (FieldUtil.isAllianceBlue() ? 0 : 180);
        double yawWithRollover = rawYaw > 0 ? rawYaw % 360 : 360 - Math.abs(rawYaw % 360);

        return Rotation2d.fromDegrees(yawWithRollover);
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

    /**
     * Will rotate the provided value by 180 if on red alliance
     */
    public void zeroGyroAdjusted(Rotation2d rot) {
        setGyroYaw(FieldUtil.isAllianceBlue() ? rot : rot.plus(Rotation2d.fromDegrees(180)));
    }

    /**
     * Will rotate the provided value by 180 if on red alliance
     */
    public void zeroGyroAdjusted() {
        setGyroYaw(FieldUtil.isAllianceBlue() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
    }

    public void updateVision() {
        if (Robot.isReal()) {

            double yaw = m_pigeon2.getYaw().getValue();
            
            LimelightHelpers.SetRobotOrientation(VisionConstants.limelightLeftName, yaw, yaw - lastYaw, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(VisionConstants.limelightCenterName, yaw, yaw - lastYaw, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(VisionConstants.limelightRightName, yaw, yaw - lastYaw, 0, 0, 0, 0);

            lastYaw = yaw;

            LimelightHelpers.PoseEstimate leftPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightLeftName);
            LimelightHelpers.PoseEstimate centerPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightCenterName);
            LimelightHelpers.PoseEstimate rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightRightName);


            if (leftPose.pose.getX() > 0.1) {
                if (leftPose.avgTagDist < VisionConstants.maxUsableDistance) {

                    double xyStdDev2 = VisionConstants.calcStdDev(leftPose.avgTagDist);

                    double timestamp = Timer.getFPGATimestamp()
                            - (leftPose.latency) / 1000;

                    // TODO Test the rotation value from the limelight pose and see if it's identical to gyro
                    Pose2d withGyroData = new Pose2d(leftPose.pose.getTranslation(), getGyroYaw()); 

                    addVisionMeasurement(withGyroData, timestamp,
                            VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
                }
            }

            if (centerPose.pose.getX() > 0.1) {
                if (centerPose.avgTagDist < VisionConstants.maxUsableDistance) {

                    double xyStdDev2 = VisionConstants.calcStdDev(centerPose.avgTagDist);

                    double timestamp = Timer.getFPGATimestamp()
                            - (centerPose.latency) / 1000;

                    Pose2d withGyroData = new Pose2d(centerPose.pose.getTranslation(), getGyroYaw());

                    addVisionMeasurement(withGyroData, timestamp,
                            VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
                }
            }

             if (rightPose.pose.getX() > 0.1) {
                if (rightPose.avgTagDist < VisionConstants.maxUsableDistance) {

                    double xyStdDev2 = VisionConstants.calcStdDev(rightPose.avgTagDist);

                    double timestamp = Timer.getFPGATimestamp()
                            - (rightPose.latency) / 1000;

                    Pose2d withGyroData = new Pose2d(rightPose.pose.getTranslation(), getGyroYaw());

                    addVisionMeasurement(withGyroData, timestamp,
                            VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
                }
            }
        }
    }

    public void periodic() {
        if (!DriverStation.isAutonomous()) {
            updateVision();
        }

        
    }

    public void resetPose(Pose2d pose) {
        this.seedFieldRelative(pose);
        this.zeroGyro(pose.getRotation());
    }

    public void configPathPlanner() {

        AutoBuilder.configureHolonomic(
                () -> PoseEstimation.getEstimatedPose(), // Robot pose supplier
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

        field = new LoggedField("PoseEstimator", logger, SwerveLogging.Pose, true);
        loggedModules = new LoggedSweveModules("Modules", logger, this,
                SwerveLogging.Modules);

        logger.add(field);
        field.addPose2d("PoseEstimation", () -> PoseEstimation.getEstimatedPose(), true);
        // logger.add(loggedModules);

        // logger.add(new LoggedPigeon2("Gyro", logger, this.m_pigeon2,
        // SwerveLogging.Gyro));

        logger.addDouble("PoseHeading", () -> PoseEstimation.getEstimatedPose().getRotation().getDegrees(),
                SwerveLogging.Pose);
        logger.addDouble("RawGyro", () -> m_pigeon2.getYaw().getValue(),
                SwerveLogging.Pose);

        // logger.addDouble("x velocity", () ->
        // PoseEstimation.getEstimatedVelocity().getX(), SwerveLogging.Pose);
        // logger.addDouble("y velocity", () ->
        // PoseEstimation.getEstimatedVelocity().getX(), SwerveLogging.Pose);

        // logger.addString("Command", () -> {
        // Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {
        // command = c.getName();
        // });
        // return command;
        // }, SwerveLogging.Main);

        // logger.addDouble("xAutoError", () ->
        // PoseEstimation.getAutoTargetPoseError().getX(), SwerveLogging.Auto);
        // logger.addDouble("yAutoError", () ->
        // PoseEstimation.getAutoTargetPoseError().getY(), SwerveLogging.Auto);
        // logger.addDouble("thetaAutoError", () ->
        // PoseEstimation.getAutoTargetPoseError().getRotation().getDegrees(),
        // SwerveLogging.Auto);

        logger.addBoolean("isAtAngularDriveSetpoint", () -> isAngularDriveAtSetpoint(), SwerveLogging.PidPose);
        // logger.addDouble("angularDriveSetpoint", () -> angularDrivePID.getSetpoint(),
        // SwerveLogging.PidPose);
        // logger.addDouble("angularDrivePositionError", () ->
        // angularDrivePID.getPositionError(), SwerveLogging.PidPose);
        // logger.addDouble("angularDriveVelocityError", () ->
        // angularDrivePID.getVelocityError(), SwerveLogging.PidPose);

        // if (SwerveLogging.PidPose == LoggingLevel.SHUFFLEBOARD) {
        // Shuffleboard.getTab(logger.getName()).add(angularDrivePID);
        // }

        logger.add(
                new LoggedFalcon("FrontLeftDrive", logger, getModule(0).getDriveMotor(), SwerveLogging.Motors, true));
        logger.add(
                new LoggedFalcon("FrontLeftAngle", logger, getModule(0).getSteerMotor(), SwerveLogging.Motors, true));

        // logger.add(
        // new LoggedFalcon("FrontRightDrive", logger, getModule(1).getDriveMotor(),
        // SwerveLogging.Motors, true));
        // logger.add(
        // new LoggedFalcon("FrontRightAngle", logger, getModule(1).getSteerMotor(),
        // SwerveLogging.Motors, true));

        // logger.add(new LoggedFalcon("BackLeftDrive", logger,
        // getModule(2).getDriveMotor(), SwerveLogging.Motors, true));
        // logger.add(new LoggedFalcon("BackLeftAngle", logger,
        // getModule(2).getSteerMotor(), SwerveLogging.Motors, true));

        // logger.add(
        // new LoggedFalcon("BackRightDrive", logger, getModule(3).getDriveMotor(),
        // SwerveLogging.Motors, true));
        // logger.add(
        // new LoggedFalcon("BackRightAngle", logger, getModule(3).getSteerMotor(),
        // SwerveLogging.Motors, true));

    }

}