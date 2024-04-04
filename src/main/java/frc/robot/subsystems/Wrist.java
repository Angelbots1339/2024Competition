// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.LoggingConstants.WristLogging;

public class Wrist extends SubsystemBase {

  private TalonFX wristLeaderMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristLeaderMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));


  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderPort);

  private final Timer throughBoreTimer = new Timer();
  private double targetPosition = 0;

  private LoggedSubsystem logger = new LoggedSubsystem("Wrist");


  /** Creates a new Wrist. */
  public Wrist() {

    wristEncoder.setDistancePerRotation(1);
    // wristEncoder.setPositionOffset(WristConstants.absoluteEncoderOffset.getRotations() % 1);

    throughBoreTimer.start();

    initializeLogging();

  }

  /**
   * PID Wrist to position
   * 
   * @param rotations 0 to 1 rotations
   */
  private void toAngle(double position) {

    wristLeaderMotor.setControl(WristConstants.wristPositionControl.withPosition(position));

    targetPosition = position;
  }

  /**
   * PID Wrist to position
   * 
   * @param rotations Rotation 2d
   */
  public void toAngle(Rotation2d position) {

    toAngle(position.getRotations());
  }

  public void home() {
    toAngle(ScoringConstants.Home.angle);
  }

  /**
   * Just PID to the current angle to hold position
   */
  public void holdPosition() {
    toAngle(getAngle());
  }

  public Rotation2d getSetpointError() {
    return Rotation2d.fromRotations(wristLeaderMotor.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError().getDegrees()) <= WristConstants.angleErrorTolerance.getDegrees();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(wristLeaderMotor.getPosition().getValue());
  }

  public double getVoltageOut() {
    return wristLeaderMotor.getMotorVoltage().getValue();
  }

  public Rotation2d getAbsoluteEncoderPosition() {
    return Rotation2d.fromRotations(wristEncoder.getAbsolutePosition()).minus(Rotation2d.fromRotations(WristConstants.absoluteEncoderOffset.getRotations()));
  }

  public void disable() {
    wristLeaderMotor.setControl(new DutyCycleOut(0));
  }

  public void setVoltage(double volts) {
    wristLeaderMotor.setControl(new VoltageOut(volts));
  }

  public void resetToAbsolute() {
    wristLeaderMotor.setPosition(getAbsoluteEncoderPosition().getRotations());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (throughBoreTimer.get() >= WristConstants.timeBeforeEncoderReset) {
      resetToAbsolute();
      throughBoreTimer.reset();
      throughBoreTimer.stop();
    }

        // System.out.println(getSetpointError());
    SmartDashboard.putNumber("WristError", getSetpointError().getDegrees());
    SmartDashboard.putNumber("WristCurrentAngle", getAngle().getDegrees());

  }

  private TalonFX configWristMotor(TalonFX motor) {

    // TODO Figure out what status signals Follower control needs to work

    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(WristConstants.kWristPositionUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(WristConstants.kWristErrorUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));

    // ErrorCheckUtil.checkError(
    //     motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
    //     CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }


  private String command = "None";
    private void initializeLogging() {

    

    logger.addString("Command", () -> {
            Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {
                command = c.getName();
            });
            return command;
        }, WristLogging.Main);

    logger.add(new LoggedFalcon("WristLeader", logger, wristLeaderMotor, WristLogging.Motor, true));

    logger.addBoolean("WristAtSetpoint", () -> isAtSetpoint(), WristLogging.Main);

    logger.addDouble("WristPosition", () -> getAngle().getDegrees(),
        WristLogging.Main);
    logger.addDouble("absEncoderPos", () -> wristEncoder.getAbsolutePosition() * 360,
        WristLogging.Main);
    logger.addDouble("absEncoderOffsetPos", () -> getAbsoluteEncoderPosition().getDegrees(),
        WristLogging.Main);

    logger.addDouble("WristVelocity",
        () -> wristLeaderMotor.getVelocity().getValue(),
        WristLogging.Main);
    logger.addDouble("WristError",
        () -> wristLeaderMotor.getClosedLoopError().getValue() * 360,
        WristLogging.Main);

    
   


  }
}
