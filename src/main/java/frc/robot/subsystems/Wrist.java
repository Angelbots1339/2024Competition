// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.math.PolynomialRegression;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.regressions.SpeakerShotRegression;

public class Wrist extends SubsystemBase {

  private TalonFX wristMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));


  /** Creates a new Wrist. */
  public Wrist() {
  }

  /**
   * PID Wrist to position
   * 
   * @param rotations 0 to 1 rotations
   */
  public void wristToPosition(double position) {

    wristMotor.setControl(WristConstants.wristPositionControl.withPosition(position));
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
        motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
        CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }
}
