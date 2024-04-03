// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.LoggingConstants.IndexerLogging;

public class Indexer extends SubsystemBase {

  private TalonFX indexerMotor = configIndexerMotor(TalonFXFactory.createTalon(IndexerConstants.indexerMotorID,
      IndexerConstants.indexerMotorCANBus, IndexerConstants.kIndexerConfiguration));

  private TimeOfFlight indexerSensor = new TimeOfFlight(IndexerConstants.indexerSensorID);

  private LoggedSubsystem logger = new LoggedSubsystem("Indexer");

  /** Creates a new Indexer. */
  public Indexer() {
    indexerSensor.setRangingMode(IndexerConstants.indexerSensorRange, IndexerConstants.indexerSampleTime);
    indexerSensor.setRangeOfInterest(8, 8, 12, 12);

    initializeLogging();
  }

  public void runIndexerDutyCycle(double speed) {
    indexerMotor.setControl(IndexerConstants.indexerDutyCycle.withOutput(speed));
  }

  public void runIndexerTorqueControl(double amps) {
    indexerMotor.setControl(IndexerConstants.indexerTorqueControl.withOutput(amps));
  }

  public void setVoltage(double volts) {
    indexerMotor.setControl(new VoltageOut(volts));
  }

  public boolean isNoteAtTarget() {
    return Math
        .abs(indexerSensor.getRange() - IndexerConstants.isNotePresentTarget) < IndexerConstants.isNotePresentTolerance;
  }

  public boolean isNotePresent() {
    return indexerSensor.getRange() < 200;
  }

  public void indexNoteToTarget() {

    if (indexerSensor.getRange() > 250) {
      setVoltage(ScoringConstants.indexingTargetVolts);
    } else if (isNoteAtTarget()) {
      disable();
    } else {
      if (indexerSensor.getRange() > IndexerConstants.isNotePresentTarget) {
        setVoltage(ScoringConstants.indexingTargetVoltsSlow);

      } else if (indexerSensor.getRange() < IndexerConstants.isNotePresentTarget) {
        setVoltage(-ScoringConstants.indexingTargetVoltsSlow);

      }
    }
  }

  public void disable() {
    indexerMotor.setControl(IndexerConstants.indexerDutyCycle.withOutput(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("IndexerSensor", indexerSensor.getRange());
  }

  private TalonFX configIndexerMotor(TalonFX motor) {

    // ErrorCheckUtil.checkError(
    // motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
    // CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }

  private String command = "None";

  private void initializeLogging() {

    

    // logger.addString("Command", () -> {
    //   Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {
    //     command = c.getName();
    //   });
    //   return command;
    // }, IndexerLogging.Main);

    logger.add(new LoggedFalcon("IndexerMotor", logger, indexerMotor, IndexerLogging.Motor, true));

    logger.addBoolean("NotePresent", () -> isNotePresent(), IndexerLogging.Main);
    logger.addBoolean("NoteAtTarget", () -> isNoteAtTarget(), IndexerLogging.Main);
    logger.addDouble("TOFSensor", () -> indexerSensor.getRange(), IndexerLogging.Main);

  }
}
