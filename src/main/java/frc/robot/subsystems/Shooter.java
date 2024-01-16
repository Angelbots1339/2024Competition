// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.math.PolynomialRegression;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.regressions.CompRegression;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotor = configShooterMotor(TalonFXFactory.createTalon(ShooterConstants.shooterMotorID,
      ShooterConstants.shooterMotorCANBus, ShooterConstants.kShooterConfiguration));


  /** Creates a new Shooter. */
  public Shooter() {

  }

  public void shooterToRMP(double rpm) {
    shooterToVelocity(rpm / 60);
  }

  public void shooterToVelocity(double speed) {
    shooterMotor.setControl(ShooterConstants.shooterControl.withVelocity(speed));
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
}
