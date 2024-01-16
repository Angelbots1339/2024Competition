// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase {

  private TalonFX intakeMotor = configIntakeMotor(TalonFXFactory.createTalon(IntakeConstants.intakeMotorID,
      IntakeConstants.intakeMotorCANBus, IntakeConstants.kIntakeConfiguration));

  /** Creates a new intake. */
  public Intake() {

  }

  public void runIntakeDutyCycle(double speed) {
    intakeMotor.setControl(IntakeConstants.intakeDutyCycle.withOutput(speed));
  }

  public void runIntakeTorqueControl(double amps) {
    intakeMotor.setControl(IntakeConstants.intakeTorqueControl.withOutput(amps));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX configIntakeMotor(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
        CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }
}
