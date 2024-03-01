// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.math.PolynomialRegression;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.FieldUtil;
import frc.lib.util.PoseEstimation;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LoggingConstants.ShooterLogging;
import frc.robot.regressions.SpeakerShotRegression;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotorLeft = configShooterMotor(TalonFXFactory.createTalon(ShooterConstants.shooterMotorLeftID,
      ShooterConstants.shooterMotorCANBus, ShooterConstants.kShooterConfiguration));
  private TalonFX shooterMotorRight = configShooterMotor(TalonFXFactory.createTalon(ShooterConstants.shooterMotorRightID,
      ShooterConstants.shooterMotorCANBus,
      ShooterConstants.kShooterConfiguration.withMotorOutput(ShooterConstants.kShooterConfiguration.MotorOutput
          .withInverted(ShooterConstants.kShooterConfiguration.MotorOutput.Inverted == InvertedValue.Clockwise_Positive
              ? InvertedValue.CounterClockwise_Positive // Make motors spin opposite directions
              : InvertedValue.Clockwise_Positive))));

  private LoggedSubsystem logger;

  private double leftTargetVelocity = 0;
  private double rightTargetVelocity = 0;

  /** Creates a new Shooter. */
  public Shooter() {

    initializeLogging();
  }

  /**
   * Set both motots to the same speed
   * 
   * @param rpm
   */
  public void shooterToRMP(double rpm) {
    shooterToVelocity(rpm / 60);
  }

  /**
   * Set shooter speeds
   * 
   * @param leftRMP
   * @param rightRPM
   */
  public void shooterToRMP(double leftRMP, double rightRPM) {
    shooterToVelocity(leftRMP / 60, rightRPM / 60);
  }

  /**
   * Set both shooter motors to the same speed
   * 
   * @param speed rotations per second
   */
  public void shooterToVelocity(double speed) {
    shooterMotorLeft.setControl(ShooterConstants.shooterControl.withVelocity(speed));
    shooterMotorRight.setControl(ShooterConstants.shooterControl.withVelocity(speed));

    leftTargetVelocity = speed;
    rightTargetVelocity = speed;
  }

  /**
   * Set shooter motors to speed
   * 
   * @param leftSpeed  rotations per second
   * @param rightSpeed rotations per second
   */
  public void shooterToVelocity(double leftSpeed, double rightSpeed) {
    shooterMotorLeft.setControl(ShooterConstants.shooterControl.withVelocity(leftSpeed));
    shooterMotorRight.setControl(ShooterConstants.shooterControl.withVelocity(rightSpeed));

    leftTargetVelocity = leftSpeed;
    rightTargetVelocity = rightSpeed;
  }

  public void setVoltage(double volts) {
    shooterMotorLeft.setControl(new VoltageOut(volts));
    shooterMotorRight.setControl(new VoltageOut(volts));

    leftTargetVelocity = 0;
    rightTargetVelocity = 0;
  }

  public boolean isAtSetpoint() {
    return Math.abs(shooterMotorLeft.getClosedLoopError().getValue()) * 60 < ShooterConstants.shooterVelocityTolerance
        && Math.abs(shooterMotorRight.getClosedLoopError().getValue()) * 60 < ShooterConstants.shooterVelocityTolerance;
  }

  /**
   * Get the current velocity of the shooter motors
   * @return [left, right]
   */
  public double[] getVelocity() {
    return new double[] { shooterMotorLeft.getVelocity().getValue(), shooterMotorRight.getVelocity().getValue() };
  }

  public double getLeftVelocity() {
    return shooterMotorLeft.getVelocity().getValue();
  }

  public double getRightVelocity() {
    return shooterMotorRight.getVelocity().getValue();
  }

  public void disable() {
    shooterMotorLeft.setControl(new DutyCycleOut(0));
    shooterMotorRight.setControl(new DutyCycleOut(0));

    leftTargetVelocity = 0;
    rightTargetVelocity = 0;
  }

  boolean isAllianceBlue = true; 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Translation2d target = FieldUtil.getAllianceSpeakerPosition();

        if (DriverStation.getAlliance().isPresent()) {
      isAllianceBlue = DriverStation.getAlliance().get() == Alliance.Blue;
    }
    // double targetDistance = PoseEstimation.getEstimatedPose().getTranslation()
    //     .getDistance(target);
        Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians(  // Find the angle to turn the robot to
        Math.atan((PoseEstimation.getEstimatedPose().getY() - target.getY())
            / (PoseEstimation.getEstimatedPose().getX() - target.getX())));

        SmartDashboard.putNumber("TargetRobotAngle", robotAngle.get().getDegrees());
        SmartDashboard.putNumber("EstimatedYFromTarget", PoseEstimation.getEstimatedPose().getY() - target.getY());
        SmartDashboard.putNumber("EstimatedXFromTarget", PoseEstimation.getEstimatedPose().getX() - target.getX());
  }

  private TalonFX configShooterMotor(TalonFX motor) {
    ErrorCheckUtil.checkError(
        motor.getVelocity().setUpdateFrequency(ShooterConstants.kShooterVelocityUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));

    // ErrorCheckUtil.checkError(
    //     motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
    //     CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }

  private void initializeLogging() {

    logger = new LoggedSubsystem("Shooter");

    logger.add(new LoggedFalcon("LeftShooterMotor", logger, shooterMotorLeft, ShooterLogging.Motor));
    logger.add(new LoggedFalcon("RightShooterMotor", logger, shooterMotorRight, ShooterLogging.Motor));

    logger.addBoolean("ShooterAtSetpoint", () -> isAtSetpoint(), ShooterLogging.Main);

    logger.addDouble("LeftShooterRPM", () -> getLeftVelocity() * 60, ShooterLogging.Main);
    logger.addDouble("RightShooterRPM", () -> getRightVelocity() * 60, ShooterLogging.Main);

    logger.addDouble("LeftShooterError", () -> shooterMotorLeft.getClosedLoopError().getValue() * 60, ShooterLogging.Main);
    logger.addDouble("RightShooterError", () -> shooterMotorRight.getClosedLoopError().getValue() * 60, ShooterLogging.Main);

  }

}
