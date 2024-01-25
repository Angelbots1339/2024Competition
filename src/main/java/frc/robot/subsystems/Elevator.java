// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMotorLeader = configElevatorMotor(TalonFXFactory.createTalon(ElevatorConstants.elevatorLeaderMotorID,
      ElevatorConstants.elevatorMotorCANBus, ElevatorConstants.kElevatorConfiguration));
  private TalonFX elevatorMotorFollower = configElevatorMotor(TalonFXFactory.createTalon(ElevatorConstants.elevatorFollowerMotorID,
      ElevatorConstants.elevatorMotorCANBus, ElevatorConstants.kElevatorConfiguration));

  private double targetHeight = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorFollower.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Move Elevator to position
   * 
   * @param meters 0 to 1 meter
   */
  public void toHeight(double height) {
    elevatorMotorLeader.setControl(
      ElevatorConstants.elevatorPositionControl.withPosition(ElevatorConstants.elevatorMetersToRotations(height)));

      targetHeight = height;
  }


  /**
   * Move elevator to home position (0)
   */
  public void home() {
    toHeight(0);
  }

  /**
   * Set all outputs to 0
   */
  public void disable() {
    elevatorMotorLeader.setControl(new DutyCycleOut(0));
  }

  /**
   * Run elevator motors at a voltage
   * 
   * @param volts
   */
  public void setElevatorVolts(double volts) {

    elevatorMotorLeader.setControl(new VoltageOut(volts));
  }

  public void resetPosition(double height) {

    elevatorMotorLeader.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
  }

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorMotorLeader.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return (targetHeight - getSetpointError()) < ElevatorConstants.heightErrorTolerance;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  private TalonFX configElevatorMotor(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(ElevatorConstants.kElevatorPositionUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(ElevatorConstants.kElevatorErrorUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));

    ErrorCheckUtil.checkError(
        motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
        CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }
}
