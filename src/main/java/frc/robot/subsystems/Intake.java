// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LoggingConstants.IntakeLogging;

public class Intake extends SubsystemBase {

  // Todo: Config Motor was leading to strange behavior, 
  private TalonFX intakeMotor = configIntakeMotor(TalonFXFactory.createTalon(IntakeConstants.intakeMotorID,
      IntakeConstants.intakeMotorCANBus, IntakeConstants.kIntakeConfiguration));
  // private TalonFX intakeMotor = new TalonFX(18, "rio");

  private TimeOfFlight intakeSensor = new TimeOfFlight(IntakeConstants.intakeSensorID);

  private LoggedSubsystem logger = new LoggedSubsystem("Intake");

  /** Creates a new intake. */
  public Intake() {

    intakeSensor.setRangingMode(IntakeConstants.intakeSensorRange, IntakeConstants.intakeSampleTime);
    intakeSensor.setRangeOfInterest(8, 8, 12, 12);

    initializeLogging();
  }

  public void disable() {
    intakeMotor.setControl(IntakeConstants.intakeDutyCycle.withOutput(0));
  }

  public void runIntakeDutyCycle(double speed) {
    intakeMotor.setControl(IntakeConstants.intakeDutyCycle.withOutput(speed));
  }

  public void runIntakeTorqueControl(double amps) {
    intakeMotor.setControl(IntakeConstants.intakeTorqueControl.withOutput(amps));
  }

  public void setVoltage(double volts) {
    intakeMotor.setControl(new VoltageOut(volts));
  }

  public boolean isNotePresent() {
    return intakeSensor.getRange() < IntakeConstants.isNotePresentThreshold;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // SmartDashboard.putNumber("IntakeSensor", intakeSensor.getRange());
  }

  private TalonFX configIntakeMotor(TalonFX motor) {

    // ErrorCheckUtil.checkError(
    //     motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
    //     CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }

  private String command = "None";
  private void initializeLogging() {

    


    //  logger.addString("Command", () -> {
    //         Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {
    //             command = c.getName();
    //         });
    //         return command;
    //     }, IntakeLogging.Main);

    logger.add(new LoggedFalcon("IntakeMotor", logger, intakeMotor, IntakeLogging.Motor, true));

    logger.addBoolean("NotePresent", () -> isNotePresent(), IntakeLogging.Main);
    logger.addDouble("TOFSensor", () -> intakeSensor.getRange(), IntakeLogging.Main);

    

  }
}
