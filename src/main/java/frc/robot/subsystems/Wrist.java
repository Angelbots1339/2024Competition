// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.math.PolynomialRegression;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.Mech2dManger;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;
import frc.robot.regressions.SpeakerShotRegression;

public class Wrist extends SubsystemBase {

  private TalonFX wristLeaderMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristLeaderMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));
  private TalonFX wristFollowerMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristFollowerMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));

    // private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(23);


  private double targetPosition = 0;

  private MechanismLigament2d simWrist;

  /** Creates a new Wrist. */
  public Wrist() {

    wristFollowerMotor.setControl(WristConstants.followerControl);

     if(Robot.isSimulation()) {
      simWrist = Mech2dManger.getInstance().getWrist();
      wristLeaderMotor.getSimState().setSupplyVoltage(12);
    }
  }

  /**
   * PID Wrist to position
   * 
   * @param rotations 0 to 1 rotations
   */
  public void wristToPosition(double position) {

    wristLeaderMotor.setControl(WristConstants.wristPositionControl.withPosition(position));
    wristFollowerMotor.setControl(WristConstants.followerControl);
    
    targetPosition = position;
  }

  /**
   * Move Wrist to position
   * 
   * @param rotations Rotation 2d
   */
  public void wristToPosition(Rotation2d position) {

    wristToPosition(position.getRotations());
  }

  public void home() {
    wristToPosition(Rotation2d.fromDegrees(90));
  }

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(wristLeaderMotor.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(targetPosition - getSetpointError()) <= ElevatorConstants.heightErrorTolerance;
  }

  public void disable() {
    wristLeaderMotor.setControl(new DutyCycleOut(0));
  }

  public void setVoltage(double volts) {
    wristLeaderMotor.setControl(new VoltageOut(volts));
  }



  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
      simWrist.setAngle(Rotation2d.fromRotations(wristLeaderMotor.getPosition().getValue() * 360));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX configWristMotor(TalonFX motor) {
    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(WristConstants.kWristPositionUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(WristConstants.kWristErrorUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));

    ErrorCheckUtil.checkError(
        motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
        CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }
}
