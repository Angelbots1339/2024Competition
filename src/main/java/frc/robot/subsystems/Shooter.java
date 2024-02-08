// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.math.PolynomialRegression;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LoggingConstants.ShooterLogging;
import frc.robot.regressions.SpeakerShotRegression;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotorTop = configShooterMotor(TalonFXFactory.createTalon(ShooterConstants.shooterMotorTopID,
      ShooterConstants.shooterMotorCANBus, ShooterConstants.kShooterConfiguration));
  private TalonFX shooterMotorBottom = configShooterMotor(TalonFXFactory.createTalon(ShooterConstants.shooterMotorTopID,
      ShooterConstants.shooterMotorCANBus,
      ShooterConstants.kShooterConfiguration.withMotorOutput(new MotorOutputConfigs()
          .withInverted(ShooterConstants.kShooterConfiguration.MotorOutput.Inverted == InvertedValue.Clockwise_Positive
              ? InvertedValue.CounterClockwise_Positive // Make motors spin opposite directions
              : InvertedValue.Clockwise_Positive)
          .withNeutralMode(ShooterConstants.kShooterConfiguration.MotorOutput.NeutralMode))));

  private LoggedSubsystem logger;

  private double topTargetVelocity = 0;
  private double bottomTargetVelocity = 0;

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
   * @param topRMP
   * @param bottomRPM
   */
  public void shooterToRMP(double topRMP, double bottomRPM) {
    shooterToVelocity(topRMP / 60, bottomRPM / 60);
  }

  /**
   * Set both shooter motors to the same speed
   * 
   * @param speed rotations per second
   */
  public void shooterToVelocity(double speed) {
    shooterMotorTop.setControl(ShooterConstants.shooterControl.withVelocity(speed));
    shooterMotorBottom.setControl(ShooterConstants.shooterControl.withVelocity(speed));

    topTargetVelocity = speed;
    bottomTargetVelocity = speed;
  }

  /**
   * Set shooter motors to speed
   * 
   * @param topSpeed    rotations per second
   * @param bottomSpeed rotations per second
   */
  public void shooterToVelocity(double topSpeed, double bottomSpeed) {
    shooterMotorTop.setControl(ShooterConstants.shooterControl.withVelocity(topSpeed));
    shooterMotorBottom.setControl(ShooterConstants.shooterControl.withVelocity(bottomSpeed));

    topTargetVelocity = topSpeed;
    bottomTargetVelocity = bottomSpeed;
  }

  public void setVoltage(double volts) {
    shooterMotorTop.setControl(new VoltageOut(volts));
    shooterMotorBottom.setControl(new VoltageOut(volts));

    topTargetVelocity = 0;
    bottomTargetVelocity = 0;
  }

  public boolean isAtSetpoint() {
    return Math.abs(shooterMotorTop.getVelocity().getValue() - topTargetVelocity) < ShooterConstants.shooterVelocityTolerance
        && Math.abs(shooterMotorBottom.getVelocity().getValue() - bottomTargetVelocity) < ShooterConstants.shooterVelocityTolerance;
  }

  public void disable() {
    shooterMotorTop.setControl(new DutyCycleOut(0));
    shooterMotorBottom.setControl(new DutyCycleOut(0));

    topTargetVelocity = 0;
    bottomTargetVelocity = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX configShooterMotor(TalonFX motor) {
    ErrorCheckUtil.checkError(
        motor.getVelocity().setUpdateFrequency(ShooterConstants.kShooterVelocityUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));

    ErrorCheckUtil.checkError(
        motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
        CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }

  private void initializeLogging() {

    logger = new LoggedSubsystem("Shooter");

    logger.add(new LoggedFalcon("TopShooterMotor", logger, shooterMotorTop, ShooterLogging.Motor));
    logger.add(new LoggedFalcon("BottomShooterMotor", logger, shooterMotorBottom, ShooterLogging.Motor));

  }

}
