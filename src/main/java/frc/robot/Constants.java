package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.WristElevatorState;
import frc.robot.subsystems.Swerve;

public class Constants {

        public static final double kConfigTimeoutSeconds = 0.1;

        public static final class SwerveConstants {

                // Both sets of gains need to be tuned to your individual robot.

                // The steer motor uses any SwerveModule.SteerRequestType control request with
                // the
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                                .withKP(100).withKI(0).withKD(0.2)
                                .withKS(0).withKV(1.5).withKA(0);
                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                                .withKP(3).withKI(0).withKD(0)
                                .withKS(0).withKV(0).withKA(0);

                // The closed-loop output type to use for the steer motors;
                // This affects the PID/FF gains for the steer motors
                private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
                // The closed-loop output type to use for the drive motors;
                // This affects the PID/FF gains for the drive motors
                private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

                // The stator current at which the wheels start to slip;
                // This needs to be tuned to your individual robot
                private static final double kSlipCurrentA = 65; 

                // Theoretical free speed (m/s) at 12v applied output;
                // This needs to be tuned to your individual robot
                public static final double kSpeedAt12VoltsMps = 6.21;

                public static final double maxSpeed = 6; // Used for driving
                public static final double maxAngularRate = 7.334783440493933; // 2 * Math.PI;

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio = 3;

                private static final double kDriveGearRatio = 5.142857142857142;
                private static final double kSteerGearRatio = 12.8;
                public static final double kWheelRadiusInches = 2;

                private static final boolean kSteerMotorReversed = false;
                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = false;

                private static final String kSwerveCANbusName = "Drivebase";
                private static final int kPigeonId = 13;

                // These are only used for simulation
                private static final double kSteerInertia = 0.00001;
                private static final double kDriveInertia = 0.001;
                // Simulated voltage necessary to overcome friction
                private static final double kSteerFrictionVoltage = 0.25;
                private static final double kDriveFrictionVoltage = 0.25;

                private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                                .withPigeon2Id(kPigeonId)
                                .withCANbusName(kSwerveCANbusName);

                private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                                .withDriveMotorGearRatio(kDriveGearRatio)
                                .withSteerMotorGearRatio(kSteerGearRatio)
                                .withWheelRadius(kWheelRadiusInches)
                                .withSlipCurrent(kSlipCurrentA)
                                .withSteerMotorGains(steerGains)
                                .withDriveMotorGains(driveGains)
                                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                                .withSteerInertia(kSteerInertia)
                                .withDriveInertia(kDriveInertia)
                                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                                .withCouplingGearRatio(kCoupleRatio)
                                .withSteerMotorInverted(kSteerMotorReversed);

                // Front Left
                private static final int kFrontLeftDriveMotorId = 7;
                private static final int kFrontLeftSteerMotorId = 9;
                private static final int kFrontLeftEncoderId = 8;
                private static final double kFrontLeftEncoderOffset = 0.398193;

                private static final double kFrontLeftXPosInches = 10.25;
                private static final double kFrontLeftYPosInches = 10.25;

                // Front Right
                private static final int kFrontRightDriveMotorId = 10;
                private static final int kFrontRightSteerMotorId = 12;
                private static final int kFrontRightEncoderId = 11;
                private static final double kFrontRightEncoderOffset = -0.252441;

                private static final double kFrontRightXPosInches = 10.25;
                private static final double kFrontRightYPosInches = -10.25;

                // Back Left
                private static final int kBackLeftDriveMotorId = 4;
                private static final int kBackLeftSteerMotorId = 6;
                private static final int kBackLeftEncoderId = 5;
                private static final double kBackLeftEncoderOffset = -0.343750;

                private static final double kBackLeftXPosInches = -10.25;
                private static final double kBackLeftYPosInches = 10.25;

                // Back Right
                private static final int kBackRightDriveMotorId = 1;
                private static final int kBackRightSteerMotorId = 3;
                private static final int kBackRightEncoderId = 2;
                private static final double kBackRightEncoderOffset = -0.253662;

                private static final double kBackRightXPosInches = -10.25;
                private static final double kBackRightYPosInches = -10.25;

                private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                                kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches),
                                Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
                private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                                kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
                                Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
                private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                                kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches),
                                Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
                private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                                kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches),
                                Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

                public static final Swerve Swerve = new Swerve(DrivetrainConstants,
                                FrontLeft,
                                FrontRight, BackLeft, BackRight);
        }

        public static final class DriverConstants {

                public static final double maxTranslationSpeed = SwerveConstants.maxSpeed;
                public static final double maxRotationSpeed = SwerveConstants.maxAngularRate;

                public static final double joystickDeadband = 0.1;
                public static final double joystickCurvePower = 1;

                public static final Boolean skewReduction = true;
                public static final Boolean openLoopDrive = true;

                public static final double looper_dt = 0.02; // loop time in seconds (used for 254's solution to swerve skew)
                public static final double FudgeFactorKp = 0.1; // used for the CD fudge factor solution to swerve skew
                public static final double FudgeFactorSimpleKp = 0.1; // used for the CD fudge factor solution to swerve
                                                                      // skew

                public static final double angularDriveKP = 0.075;
                public static final double angularDriveKI = 0;
                public static final double angularDriveKD = 0.005;
                public static final double angularDriveKS = 0.4; // radians per sec
                public static final double angularDriveTolerance = 2; // Degrees

                public static final double pidToPoseKP = 2.5;
                public static final double pidToPoseKD = 0;
                public static final double pidToPoseKS = 0.15;
                public static final double pidToPoseTolerance = 0.03; // Meters
                public static final double pidToPoseMaxSpeed = 1; // Meters per second

                public static final double manualElevatorVolts = 10;
                public static final double manualWristVolts = 6;

                public static final double endgameAlert1 = 20; // Seconds
                public static final double endgameAlert2 = 10; // Seconds

                /**
                 * 
                 * @param val         The joystick value
                 * @param curveInputs Whether or not to curve the inputs (x^power)
                 * @return Desired robot speed from the joystick value
                 */
                public static double fixTranslationJoystickValues(double val, boolean curveInputs) {
                        return MathUtil.applyDeadband(Math.pow(Math.abs(val), curveInputs ? joystickCurvePower : 1),
                                        joystickDeadband) * maxTranslationSpeed * Math.signum(val);
                }

                /**
                 * 
                 * @param val         The joystick value
                 * @param curveInputs Whether or not to curve the inputs (x^power)
                 * @return Desired robot speed from the joystick value
                 */
                public static double fixRotationJoystickValues(double val, boolean curveInputs) {
                        return MathUtil.applyDeadband(Math.pow(Math.abs(val), curveInputs ? joystickCurvePower : 1),
                                        joystickDeadband) * maxRotationSpeed * Math.signum(val);
                }
        }

        public static final class AutoConstants {

                public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
                                // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants in Meters
                                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants in Radians
                                SwerveConstants.kSpeedAt12VoltsMps, // Max module speed, in m/s
                                0.40451045104, // Drive base radius in meters. Distance from robot center to furthest
                                               // module.
                                new ReplanningConfig() // Default path replanning config. See the API for the options
                                                       // here
                );
        }

        public static final class ShooterConstants {

                public static final int shooterMotorLeftID = 16; 
                public static final int shooterMotorRightID = 17;
                public static final String shooterMotorCANBus = "";

                public static final double shooterGearRatio = 0.5; // Sensor to Mechanism Ratio

                public static final double shooterVelocityTolerance = 500; // RPM

                public static final TalonFXConfiguration kShooterConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(120)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimitEnable(false))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.CounterClockwise_Positive))
                                .withSlot0(new Slot0Configs()
                                                .withKV(0.071) // 0.075
                                                .withKP(0.5) // 0.125
                                                .withKI(0)
                                                .withKD(0))
                                .withFeedback(new FeedbackConfigs()
                                                .withSensorToMechanismRatio(shooterGearRatio));

                public static final VelocityVoltage shooterControl = new VelocityVoltage(0, 0, false, 0, 0, false,
                                false, false);

                public static final double kShooterVelocityUpdateFrequency = 10; // Hertz

                // public static final double gamePieceSpeedLeavingShooter = 2; // Meters/second
        }

        public static final class IntakeConstants {
                public static final int intakeMotorID = 18;
                public static final String intakeMotorCANBus = "";

                public static final int intakeSensorID = 26;
                public static final RangingMode intakeSensorRange = RangingMode.Short;
                public static final double intakeSampleTime = 24;

                public static final double isNotePresentThreshold = 250; // Milimeters

                private static final double intakeMaxDutyCycle = 0.5;

                public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(false)
                                                .withSupplyCurrentLimitEnable(false))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.Clockwise_Positive));

                public static final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0, true, false,
                                false, false);

                public static final TorqueCurrentFOC intakeTorqueControl = new TorqueCurrentFOC(0, intakeMaxDutyCycle,
                                0, false, false,
                                false);

        }

        public static final class IndexerConstants {
                public static final int indexerMotorID = 19; 
                public static final String indexerMotorCANBus = "";

                public static final int indexerSensorID = 27; 
                public static final RangingMode indexerSensorRange = RangingMode.Short;
                public static final double indexerSampleTime = 24;

                public static final double isNotePresentTarget = 70; // Milimeters
                public static final double isNotePresentTolerance = 5; // Milimeters

                private static final double indexerMaxDutyCycle = 0.5;

                public static final TalonFXConfiguration kIndexerConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.CounterClockwise_Positive));

                public static final DutyCycleOut indexerDutyCycle = new DutyCycleOut(0, true, false,
                                false, false);

                public static final TorqueCurrentFOC indexerTorqueControl = new TorqueCurrentFOC(0, indexerMaxDutyCycle,
                                0, false,
                                false,
                                false);

        }

        public static final class WristConstants {
                public static final int wristLeaderMotorID = 20; 
                public static final int wristFollowerMotorID = 21; 
                public static final String wristMotorCANBus = "";

                public static final double wristGearRatio = 15 * 2; // Sensor to Mechanism Ratio
                public static final double timeBeforeEncoderReset = 1.5; // Seconds before the motor is initialized to
                // the through bore;

                public static final int wristEncoderPort = 0; 
                public static final Rotation2d absoluteEncoderOffset = Rotation2d.fromDegrees(105); // 0 Should be
                                                                                                    // straight forward
                                                                                                    // towards the
                                                                                                    // intake

                /****
                 * Wrist is counter-clockwise (from the left side of the robot with intake
                 * forward) with 0 being horizontal towards the intake
                 ****/
                public static final Rotation2d wristMinAngle = Rotation2d.fromDegrees(-25);
                public static final Rotation2d wristMaxAngle = Rotation2d.fromDegrees(180);

                public static final TalonFXConfiguration kWristConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withSlot0(new Slot0Configs()
                                                .withKV(0)
                                                .withKA(0)
                                                .withKP(55) // 40  || 55
                                                .withKI(0)
                                                .withKD(5) // 0.5 || 5
                                                .withGravityType(GravityTypeValue.Arm_Cosine)
                                                .withKG(-0.4) // Negative b/c of wrist direction & how CTRE uses it
                                                .withKS(0.5)) // 0.2
                                .withFeedback(new FeedbackConfigs()
                                                .withSensorToMechanismRatio(wristGearRatio))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(10) // Default 10
                                                .withMotionMagicAcceleration(15) // Default 15
                                                .withMotionMagicJerk(0))
                                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                                .withForwardSoftLimitEnable(true)
                                                .withReverseSoftLimitEnable(true)
                                                .withForwardSoftLimitThreshold(wristMaxAngle.getRotations())
                                                .withReverseSoftLimitThreshold(wristMinAngle.getRotations()));

                public static final MotionMagicVoltage wristPositionControl = new MotionMagicVoltage(0, false, 0, 0,
                                false,
                                false, false);
                // public static final PositionVoltage wristPositionControl = new
                // PositionVoltage(0, 0, true, 0, 0, false, false, false);

                public static final Follower followerControl = new Follower(wristLeaderMotorID, true);

                public static final double kWristPositionUpdateFrequency = 10; // Hertz
                public static final double kWristErrorUpdateFrequency = 20; // Hertz

                public static final Rotation2d angleErrorTolerance = Rotation2d.fromDegrees(0.5); // Degrees

        }

        public static final class ElevatorConstants {
                public static final int elevatorLeaderMotorID = 22; 
                public static final int elevatorFollowerMotorID = 23;
                public static final String elevatorMotorCANBus = "";

                public static final double elevatorGearRatio = 25; // Sensor to Mechanism Ratio
                public static final double elevatorPinionRadius = Units.inchesToMeters(1); // Meters

                public static final double maxElevatorHeight = 0.46; // Meters

                public static final TalonFXConfiguration kElevatorConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(false)
                                                .withSupplyCurrentLimitEnable(false))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withSlot0(new Slot0Configs()
                                                .withKV(0)
                                                .withKA(0)
                                                .withKP(15)
                                                .withKI(0)
                                                .withKD(0)
                                                .withGravityType(GravityTypeValue.Elevator_Static)
                                                .withKG(0.35))
                                .withFeedback(new FeedbackConfigs()
                                                .withSensorToMechanismRatio(elevatorGearRatio))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(6)
                                                .withMotionMagicAcceleration(18)
                                                .withMotionMagicJerk(0))
                                .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
                                                .withForwardLimitEnable(true)
                                                .withReverseLimitEnable(true)
                                                .withForwardLimitAutosetPositionEnable(false)
                                                .withReverseLimitAutosetPositionEnable(false)
                                                .withForwardLimitAutosetPositionValue(maxElevatorHeight)
                                                .withReverseLimitAutosetPositionValue(0)
                                                .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
                                                .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
                                                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                                                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen))
                                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                                .withForwardSoftLimitEnable(true)
                                                .withReverseSoftLimitEnable(false)
                                                .withForwardSoftLimitThreshold(
                                                                elevatorMetersToRotations(maxElevatorHeight))
                                                .withReverseSoftLimitThreshold(0));

                public static final MotionMagicVoltage elevatorPositionControl = new MotionMagicVoltage(0, true, 0, 0,
                                false,
                                false, false);

                public static final Follower followerControl = new Follower(elevatorLeaderMotorID, true);

                public static final double heightErrorTolerance = 0.005; // Meters

                public static final double kElevatorPositionUpdateFrequency = 10; // Hertz
                public static final double kElevatorErrorUpdateFrequency = 20; // Hertz

                public static double elevatorMetersToRotations(double meters) {

                        return meters / (2 * Math.PI * elevatorPinionRadius);
                }

                public static double elevatorRotationsToMeters(double rotations) {

                        return rotations * (2 * Math.PI * elevatorPinionRadius);
                }
        }

        public static class ScoringConstants {

                public static final WristElevatorState Handoff = new WristElevatorState(138, 0);
                public static final WristElevatorState ScoreAmp = new WristElevatorState(18, 0.25);
                public static final WristElevatorState Home = new WristElevatorState(90, 0);
                public static final WristElevatorState SubwooferShot = new WristElevatorState(127, 0);

                public static final double[] shooterSetpointClose = { 3750, 4750 }; // [Left, Right]
                public static final double[] shooterSetpointFar = { 5000, 6000 }; // [Left, Right]
                public static final double flywheelDistanceCutoff = 2.75; // 2.9
                public static final double shootingWaitTime = 0.2; // Seconds to wait before shooting to make shooting
                                                                   // predictable & consistent

                public static final double wristRegressionMaxClamp = 175;
                public static final double wristRegressionMinClamp = 90;

                public static final double scoreAmpOffset = 0.4;

                public static final double indexingTargetPercent = 0.4;
                // public static final double indexingTargetPercentSlow = 0.1;

                public static final double indexingTargetVolts = 5; 
                public static final double indexingTargetVoltsSlow = 1.5;
                public static final double indexerScoringVoltage = 5;

                public static final double intakingTargetVoltage = 6;
                public static final double outtakingTargetVoltage = -6;

                public static final double kAccelCompFactor = 0; // Seconds
                public static final double gamePieceVelocity = 0; // Meters per second

                public static final double shootingDriveScalar = 0.25;

                public static final boolean shootWhileMoving = false;

                public static final double autonomousFinishShotTime = 0.3;

        }

        public static final class VisionConstants {

                // Desmos plots:
                // \left[\left(110,.0088\right),\left(101.75,.01065\right),\left(94,.0088\right),\left(87.25,.0053\right),\left(78.75,.0028\right),\left(64,.0037\right),\left(49.5,.0036\right),\left(113.25,.0124\right),\left(136.75,.34\right),\left(153.5,.387\right),\left(160.5,.3342\right),\left(167,1.573\right),\left(121.25,.052\right)\right]
                // Apriltag Height: 57 inches
                // Horizontal offset 22.5 inches

                // public static final double StdDevScalar = 0.002;

                public static double calcStdDev(double metersFromTarget) {

                        // double inches = Units.metersToInches(metersFromTarget); // Convert to inches
                        // double hypotenuse = Math.sqrt(Math.pow(inches, 2) + Math.pow(57, 2)); //
                        // Account for april tag being high off the ground
                        // double calculated = StdDevScalar * Math.pow(Math.pow(hypotenuse, 19.5335),
                        // -0.1/2); // Plug into Std Dev equation that we got experimentally

                        // return MathUtil.clamp(0.01 * Math.pow(metersFromTarget, 3), 0, 1);
                        return 0.165 * Math.pow(metersFromTarget, 2);
                }

                public static final double maxUsableDistance = 5.25; // Meters

                public static final String limelightLeftName = "limelight-left";
                public static final String limelightCenterName = "limelight-center";
                public static final String limelightRightName = "limelight-right";

                // public static final Translation3d limelightLeftOffset = new Translation3d(
                //                 Units.inchesToMeters(-11.384098),
                //                 Units.inchesToMeters(-8.607840), Units.inchesToMeters(14.617002));
                // public static final Translation3d limelightCenterOffset = new Translation3d(
                //                 Units.inchesToMeters(-9.877464),
                //                 Units.inchesToMeters(0), Units.inchesToMeters(10.493215));
                // public static final Translation3d limelightRightOffset = new Translation3d(
                //                 Units.inchesToMeters(-11.384098),
                //                 Units.inchesToMeters(8.607840), Units.inchesToMeters(14.617002));

                
    
                // New Mounts with correctly rotated limelight:

                // Angled: X: 0.3013935806 Y: 0.2255540828 Z: 0.3013935806

                // Limelight left rotation: Pitch: 10 Roll: 0 Yaw: 40.5
                // Limelight center rotation: Pitch:24 Roll: Yaw:
                // Limelight right rotation: Pitch:10 Roll: Yaw: -40.5

        }

}
