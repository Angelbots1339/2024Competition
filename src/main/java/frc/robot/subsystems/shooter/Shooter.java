// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotor = TalonFXFactory.createTalon(ShooterConstants.shooterMotorID, ShooterConstants.shooterMotorCANBus, ShooterConstants.kShooterConfiguration);

  /** Creates a new Shooter. */
  public Shooter() {

  }

  public void shooterToRMP() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configShooterMotor() {
    ErrorCheckUtil.checkError(shooterMotor.getVelocity().setUpdateFrequency(ShooterConstants.kShooterVelocityUpdateFrequency, Constants.kConfigTimeoutSeconds),
        "Problem setting update frequency on Talon " + shooterMotor.getDeviceID());
  }
}
