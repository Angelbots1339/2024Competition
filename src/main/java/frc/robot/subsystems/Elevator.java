// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.LoggingConstants.ElevatorLogging;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorLeaderMotor = configElevatorMotor(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorLeaderMotorID,
          ElevatorConstants.elevatorMotorCANBus, ElevatorConstants.kElevatorConfiguration));

  private TalonFX elevatorFollowerMotor = configElevatorMotor(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorFollowerMotorID,
          ElevatorConstants.elevatorMotorCANBus, ElevatorConstants.kElevatorConfiguration));

  private double targetHeight = 0;
  private boolean reverseLimitHit = false;

  private LoggedSubsystem logger = new LoggedSubsystem("Elevator");

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorFollowerMotor.setControl(ElevatorConstants.followerControl);

    initializeLogging();
  }

  /**
   * Move Elevator to position
   * 
   * @param height in meters (0 to max height)
   */
  public void toHeight(double height) {

    // if (isAtSetpoint() && height == 0) { // TODO Test this
    //   elevatorLeaderMotor.setControl(new DutyCycleOut(0));
    // } else {
    //   elevatorLeaderMotor.setControl(
    //       ElevatorConstants.elevatorPositionControl.withPosition(ElevatorConstants.elevatorMetersToRotations(height)));
    // }

    elevatorLeaderMotor.setControl(
          ElevatorConstants.elevatorPositionControl.withPosition(ElevatorConstants.elevatorMetersToRotations(height)));

    elevatorFollowerMotor.setControl(ElevatorConstants.followerControl);

    targetHeight = height;
  }

  /**
   * Move elevator to home position (0)
   */
  public void home() {
    toHeight(ScoringConstants.Home.height);
  }

  public void holdPosition() {
    elevatorLeaderMotor.setControl(new VoltageOut(ElevatorConstants.kElevatorConfiguration.Slot0.kG));
    elevatorFollowerMotor.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Set all outputs to 0
   */
  public void disable() {
    elevatorLeaderMotor.setControl(new DutyCycleOut(0));
    // elevatorFollowerMotor.setControl(new DutyCycleOut(0));
  }

  /**
   * Run elevator motors at a voltage
   * 
   * @param volts
   */
  public void setVoltage(double volts) {

    elevatorLeaderMotor.setControl(new VoltageOut(volts));
    // elevatorFollowerMotor.setControl(new VoltageOut(volts));
    elevatorFollowerMotor.setControl(ElevatorConstants.followerControl);

  }

  public void resetEncoderPosition(double height) {
    elevatorLeaderMotor.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
    elevatorFollowerMotor.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
  }

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderMotor.getClosedLoopError().getValue());
  }

  public double getLeaderPosition() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderMotor.getPosition().getValue());
  }

  public double getFollowerPosition() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorFollowerMotor.getPosition().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError()) < ElevatorConstants.heightErrorTolerance;
  }

  public boolean getBottomLimitSwitch() {
    return elevatorLeaderMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println(getBottomLimitSwitch());

    if (getBottomLimitSwitch() && !reverseLimitHit) {
      elevatorLeaderMotor.setPosition(0, 0.001);
      elevatorFollowerMotor.setPosition(0, 0.001);

      reverseLimitHit = true;
    } else if (reverseLimitHit && !getBottomLimitSwitch()) {
      reverseLimitHit = false;
    }

    // SmartDashboard.putBoolean("ElevatorAtSetpoint", isAtSetpoint());
    // SmartDashboard.putNumber("ElevatorAtSetpoint", targetHeight);

  }

  private TalonFX configElevatorMotor(TalonFX motor) {

    // TODO Figure out what status signals Follower control needs to work

    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getStatorCurrent().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getReverseLimit().setUpdateFrequency(ElevatorConstants.kElevatorFastUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));

    // ErrorCheckUtil.checkError(
    // motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
    // CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }

  private String command = "None";

  private void initializeLogging() {

    // logger.addString("Command", () -> {
    // Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {
    // command = c.getName();
    // });
    // return command;
    // }, ElevatorLogging.Main);

    logger.add(new LoggedFalcon("ElevatorLeader", logger, elevatorLeaderMotor, ElevatorLogging.Motor, true));
    // logger.add(new LoggedFalcon("ElevatorFollower", logger,
    // elevatorFollowerMotor, ElevatorLogging.Motor, true));

    logger.addBoolean("ElevatorAtSetpoint", () -> isAtSetpoint(), ElevatorLogging.Main);
    logger.addBoolean("ElevatorBottomLimit", () -> getBottomLimitSwitch(), ElevatorLogging.Main);

    // logger.addDouble("ElevatorPosition", () ->
    // ElevatorConstants.elevatorRotationsToMeters(getLeaderPosition()),
    // ElevatorLogging.Main);
    // logger.addDouble("ElevatorVelocity",
    // () ->
    // ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderMotor.getVelocity().getValue()),
    // ElevatorLogging.Main);

  }
}
