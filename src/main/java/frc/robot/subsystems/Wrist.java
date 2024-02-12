// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.Mech2dManger;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;

public class Wrist extends SubsystemBase {

  private TalonFX wristLeaderMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristLeaderMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));
  private TalonFX wristFollowerMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristFollowerMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));

  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(23);

  private final Timer throughBoreTimer = new Timer();
  private double targetPosition = 0;

  private MechanismLigament2d simWrist;

  /** Creates a new Wrist. */
  public Wrist() {

    wristFollowerMotor.setControl(WristConstants.followerControl);

    wristEncoder.setDistancePerRotation(1);

    if (Robot.isSimulation()) {
      simWrist = Mech2dManger.getInstance().getWrist();
      wristLeaderMotor.getSimState().setSupplyVoltage(12);
    }
  }

  /**
   * PID Wrist to position
   * 
   * @param rotations 0 to 1 rotations
   */
  public void toAngle(double position) {

    wristLeaderMotor.setControl(WristConstants.wristPositionControl.withPosition(position));
    wristFollowerMotor.setControl(WristConstants.followerControl);

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

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(wristLeaderMotor.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(targetPosition - getSetpointError()) <= ElevatorConstants.heightErrorTolerance;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(wristLeaderMotor.getPosition().getValue());
  }

  public Rotation2d getAbsoluteEncoderPosition() {
    return Rotation2d.fromRotations(wristEncoder.getAbsolutePosition());
  }

  public void disable() {
    wristLeaderMotor.setControl(new DutyCycleOut(0));
  }

  public void setVoltage(double volts) {
    wristLeaderMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void simulationPeriodic() {
    simWrist.setAngle(Rotation2d.fromRotations(wristLeaderMotor.getPosition().getValue() * 360));
  }

  public void resetToAbsolute() {
    wristLeaderMotor.setPosition(WristConstants.absoluteEncoderOffset.getRotations() - wristEncoder.getAbsolutePosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (throughBoreTimer.get() >= WristConstants.timeBeforeEncoderReset) {
      resetToAbsolute();
      throughBoreTimer.reset();
      throughBoreTimer.stop();
    }
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
