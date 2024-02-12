// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.Mech2dManger;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoringConstants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMotorLeader = configElevatorMotor(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorLeaderMotorID,
          ElevatorConstants.elevatorMotorCANBus, ElevatorConstants.kElevatorConfiguration));
  private TalonFX elevatorMotorFollower = configElevatorMotor(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorFollowerMotorID,
          ElevatorConstants.elevatorMotorCANBus, ElevatorConstants.kElevatorConfiguration));

  private TalonFXSimState leaderSim = elevatorMotorLeader.getSimState();
  private TalonFXSimState followerSim = elevatorMotorFollower.getSimState();

  private MechanismLigament2d elevatorMech;
  private ElevatorSim elevatorSim;

  private double targetHeight = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorFollower.setControl(ElevatorConstants.followerControl);

    if (Robot.isSimulation()) {
      elevatorMech = Mech2dManger.getInstance().getElevator();

      elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2), ElevatorConstants.elevatorGearRatio, 20,
          ElevatorConstants.elevatorPinionRadius, 0, 1.4732, true, 0);
    }
  }

  /**
   * Move Elevator to position
   * 
   * @param height in meters (0 to max height)
   */
  public void toHeight(double height) {

    if (!isAtSetpoint()) {
      elevatorMotorLeader.setControl(
          ElevatorConstants.elevatorPositionControl.withPosition(ElevatorConstants.elevatorMetersToRotations(height)));
    } else { 
      // Let the elevator rest while at 0 (stop outputting Kg)
      elevatorMotorLeader.setControl(new DutyCycleOut(0));
    }

    elevatorMotorFollower.setControl(ElevatorConstants.followerControl);

    targetHeight = height;
  }

  /**
   * Move elevator to home position (0)
   */
  public void home() {
      toHeight(ScoringConstants.Home.height);
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
  public void setVoltage(double volts) {

    elevatorMotorLeader.setControl(new VoltageOut(volts));
  }

  public void resetEncoderPosition(double height) {
    elevatorMotorLeader.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
  }

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorMotorLeader.getClosedLoopError().getValue());
  }

  public double getPosition() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorMotorLeader.getPosition().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(targetHeight - getSetpointError()) < ElevatorConstants.heightErrorTolerance;
  }

  @Override
  public void simulationPeriodic() {
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    elevatorSim.setInputVoltage(leaderSim.getMotorVoltage());
    elevatorSim.update(0.02);
    elevatorMech.setLength(elevatorSim.getPositionMeters());

    leaderSim.setRawRotorPosition(ElevatorConstants.elevatorMetersToRotations(elevatorSim.getPositionMeters()));
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
