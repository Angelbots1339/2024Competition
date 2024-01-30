// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private TalonFX intakeMotor = configIndexerMotor(TalonFXFactory.createTalon(IndexerConstants.indexerMotorID,
      IndexerConstants.indexerMotorCANBus, IndexerConstants.kIndexerConfiguration));

  /** Creates a new Indexer. */
  public Indexer() {

  }

  public void runIndexerDutyCycle(double speed) {
    intakeMotor.setControl(IndexerConstants.indexerDutyCycle.withOutput(speed));
  }

  public void runIndexerTorqueControl(double amps) {
    intakeMotor.setControl(IndexerConstants.indexerTorqueControl.withOutput(amps));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX configIndexerMotor(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
        CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }
}
