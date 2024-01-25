package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve;

public class Constants {

        public static final double kConfigTimeoutSeconds = 0.1;

        public static final class GeneratedSwerveConstants {

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
                private static final double kSlipCurrentA = 300.0; // TODO Get a test value

                // Theoretical free speed (m/s) at 12v applied output;
                // This needs to be tuned to your individual robot
                public static final double kSpeedAt12VoltsMps = 6.21;

                public static final double maxSpeed = 6; // Used for driving
                public static final double maxAngularRate = 1.5 * Math.PI;

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio = 3;

                private static final double kDriveGearRatio = 5.142857142857142;
                private static final double kSteerGearRatio = 12.8;
                public static final double kWheelRadiusInches = 2;

                private static final boolean kSteerMotorReversed = false;
                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;

                private static final String kSwerveCANbusName = "";
                private static final int kPigeonId = 21;

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
                private static final int kFrontLeftDriveMotorId = 1;
                private static final int kFrontLeftSteerMotorId = 3;
                private static final int kFrontLeftEncoderId = 2;
                private static final double kFrontLeftEncoderOffset = -0.995361328125;

                private static final double kFrontLeftXPosInches = 10.75;
                private static final double kFrontLeftYPosInches = 11.75;

                // Front Right
                private static final int kFrontRightDriveMotorId = 12;
                private static final int kFrontRightSteerMotorId = 14;
                private static final int kFrontRightEncoderId = 13;
                private static final double kFrontRightEncoderOffset = -0.443359375;

                private static final double kFrontRightXPosInches = 10.75;
                private static final double kFrontRightYPosInches = -11.75;

                // Back Left
                private static final int kBackLeftDriveMotorId = 4;
                private static final int kBackLeftSteerMotorId = 6;
                private static final int kBackLeftEncoderId = 5;
                private static final double kBackLeftEncoderOffset = -0.334228515625;

                private static final double kBackLeftXPosInches = -10.75;
                private static final double kBackLeftYPosInches = 11.75;

                // Back Right
                private static final int kBackRightDriveMotorId = 9;
                private static final int kBackRightSteerMotorId = 11;
                private static final int kBackRightEncoderId = 10;
                private static final double kBackRightEncoderOffset = -0.608642578125;

                private static final double kBackRightXPosInches = -10.75;
                private static final double kBackRightYPosInches = -11.75;

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

                public static final double maxTranslationSpeed = GeneratedSwerveConstants.maxSpeed;
                public static final double maxRotationSpeed = GeneratedSwerveConstants.maxAngularRate;

                public static final double joystickDeadband = 0.1;
                public static final double joystickCurvePower = 1.5;

                public static final Boolean skewReduction = true;
                public static final Boolean openLoopDrive = true;

                public static final double looper_dt = 0.02; // used for 254's solution to swerve skew it is loop time in sec
                public static final double FudgeFactorKp = 0.1; // used for the CD fudge factor solution to swerve skew
                public static final double FudgeFactorSimpleKp = 0.1; // used for the CD fudge factor solution to swerve skew

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
                                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                                GeneratedSwerveConstants.kSpeedAt12VoltsMps, // Max module speed, in m/s
                                0.40451045104, // Drive base radius in meters. Distance from robot center to furthest
                                               // module.
                                new ReplanningConfig() // Default path replanning config. See the API for the options
                                                       // here
                );
        }

        public static final class ShooterConstants {

                public static final int shooterMotorID = 16; // TODO Set these
                public static final String shooterMotorCANBus = "canivore";

                public static final TalonFXConfiguration kShooterConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withSlot0(new Slot0Configs()
                                                .withKP(0)
                                                .withKI(0)
                                                .withKD(0))
                                .withFeedback(new FeedbackConfigs()
                                                .withSensorToMechanismRatio(1));

                public static final VelocityVoltage shooterControl = new VelocityVoltage(0, 0, false, 0, 0, false,
                                false, false);

                public static final double kShooterVelocityUpdateFrequency = 10; // Hertz

                public static final double gamePieceSpeedLeavingShooter = 2; // Meters/second
        }

        public static final class IntakeConstants {
                public static final int intakeMotorID = 17; // TODO Set these
                public static final String intakeMotorCANBus = "canivore";

                private static final double intakeMaxSpeed = 0.5;

                public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.Clockwise_Positive));

                public static final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0, true, false,
                                false, false);

                public static final TorqueCurrentFOC intakeTorqueControl = new TorqueCurrentFOC(0, 0.5, 0, false, false,
                                false);

        }

        public static final class WristConstants {
                public static final int wristMotorID = 18; // TODO Set these
                public static final String wristMotorCANBus = "canivore";

                public static final double wristGearRatio = 1; // Sensor to Mechanism Ratio

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
                                                .withKP(0)
                                                .withKI(0)
                                                .withKD(0)
                                                .withGravityType(GravityTypeValue.Arm_Cosine)
                                                .withKG(0))
                                .withFeedback(new FeedbackConfigs()
                                                .withSensorToMechanismRatio(wristGearRatio))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(0) // TODO Tune
                                                .withMotionMagicAcceleration(0)
                                                .withMotionMagicJerk(0));

                public static final MotionMagicVoltage wristPositionControl = new MotionMagicVoltage(0, true, 0, 0,
                                false,
                                false, false);

                public static final double kWristPositionUpdateFrequency = 10; // Hertz
                public static final double kWristErrorUpdateFrequency = 20; // Hertz

        }

        public static final class ElevatorConstants {
                public static final int elevatorLeaderMotorID = 19; // TODO Set these
                public static final int elevatorFollowerMotorID = 20;
                public static final String elevatorMotorCANBus = "canivore";

                public static final double elevatorGearRatio = 15; // Sensor to Mechanism Ratio
                public static final double elevatorPinionRadius = 0.05; // Meters

                public static final TalonFXConfiguration kElevatorConfiguration = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withSupplyCurrentLimit(40)
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withSlot0(new Slot0Configs()
                                                .withKP(0)
                                                .withKI(0)
                                                .withKD(0)
                                                .withGravityType(GravityTypeValue.Elevator_Static)
                                                .withKG(0))
                                .withFeedback(new FeedbackConfigs()
                                                .withSensorToMechanismRatio(elevatorGearRatio))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(0) // TODO Tune
                                                .withMotionMagicAcceleration(0)
                                                .withMotionMagicJerk(0))
                                .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
                                                .withForwardLimitEnable(true)
                                                .withReverseLimitEnable(true)
                                                .withForwardLimitAutosetPositionEnable(false)
                                                .withReverseLimitAutosetPositionEnable(false)
                                                .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
                                                .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
                                                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                                                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen));

                public static final MotionMagicVoltage elevatorPositionControl = new MotionMagicVoltage(0, true, 0, 0,
                                false,
                                false, false);

                public static final Follower followerControl = new Follower(elevatorLeaderMotorID, true);

                public static final double heightErrorTolerance = 0.02; // Meters

                public static final double kElevatorPositionUpdateFrequency = 10; // Hertz
                public static final double kElevatorErrorUpdateFrequency = 20; // Hertz



                public static double elevatorMetersToRotations(double meters) {

                        return meters / (2 * Math.PI * elevatorPinionRadius);
                }

                public static double elevatorRotationsToMeters(double rotations) {

                        return rotations * (2 * Math.PI * elevatorPinionRadius);
                }
        }

        public static final class VisionConstants {

                public static final String limelight1Name = "";

                public static final Translation3d limelight1Offset = new Translation3d(Units.inchesToMeters(0),
                                Units.inchesToMeters(0), Units.inchesToMeters(0));
        }

}
