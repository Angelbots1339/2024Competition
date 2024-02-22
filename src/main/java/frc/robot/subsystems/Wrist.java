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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.ErrorCheckUtil.CommonErrorNames;
import frc.lib.util.Mech2dManger;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.LoggingConstants.WristLogging;
import frc.robot.Robot;

public class Wrist extends SubsystemBase {

  private TalonFX wristLeaderMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristLeaderMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));
  private TalonFX wristFollowerMotor = configWristMotor(TalonFXFactory.createTalon(WristConstants.wristFollowerMotorID,
      WristConstants.wristMotorCANBus, WristConstants.kWristConfiguration));

  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderPort);

  private final Timer throughBoreTimer = new Timer();
  private double targetPosition = 0;

  private MechanismLigament2d simWrist;
  private LoggedSubsystem logger;


  /** Creates a new Wrist. */
  public Wrist() {

    wristFollowerMotor.setControl(WristConstants.followerControl);

    wristEncoder.setDistancePerRotation(1);
    // wristEncoder.setPositionOffset(WristConstants.absoluteEncoderOffset.getRotations() % 1);

    throughBoreTimer.start();

    initializeLogging();

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

  public Rotation2d getSetpointError() {
    return Rotation2d.fromRotations(wristLeaderMotor.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError().getDegrees()) <= WristConstants.angleErrorTolerance.getDegrees();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(wristLeaderMotor.getPosition().getValue());
  }

  public double getVoltageOut() {
    return wristLeaderMotor.getMotorVoltage().getValue();
  }

  public Rotation2d getAbsoluteEncoderPosition() {
    return Rotation2d.fromRotations(wristEncoder.getAbsolutePosition()).minus(Rotation2d.fromRotations(WristConstants.absoluteEncoderOffset.getRotations()));
  }

  public void disable() {
    wristLeaderMotor.setControl(new DutyCycleOut(0));
  }

  public void setVoltage(double volts) {
    wristLeaderMotor.setControl(new VoltageOut(volts));
    wristFollowerMotor.setControl(WristConstants.followerControl);
  }

  @Override
  public void simulationPeriodic() {
    simWrist.setAngle(Rotation2d.fromRotations(wristLeaderMotor.getPosition().getValue() * 360));
  }

  public void resetToAbsolute() {
    wristLeaderMotor.setPosition(getAbsoluteEncoderPosition().getRotations());
    wristFollowerMotor.setPosition(getAbsoluteEncoderPosition().getRotations());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (throughBoreTimer.get() >= WristConstants.timeBeforeEncoderReset) {
      resetToAbsolute();
      throughBoreTimer.reset();
      throughBoreTimer.stop();
    }

    // SmartDashboard.putNumber("Through Bore Transformed", getAbsoluteEncoderPosition().getDegrees());
    // SmartDashboard.putNumber("Wrist Position", getAngle().getDegrees());

    // SmartDashboard.putNumber("WristSetpoint", wristLeaderMotor.getClosedLoopReference().getValue());
    // SmartDashboard.putBoolean("WristAtSetpoint", isAtSetpoint());

  }

  private TalonFX configWristMotor(TalonFX motor) {

    // TODO Figure out what status signals Follower control needs to work

    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(WristConstants.kWristPositionUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(WristConstants.kWristErrorUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));

    // ErrorCheckUtil.checkError(
    //     motor.optimizeBusUtilization(Constants.kConfigTimeoutSeconds),
    //     CommonErrorNames.OptimizeBusUtilization(motor.getDeviceID()));

    return motor;
  }

    private void initializeLogging() {

    logger = new LoggedSubsystem("Wrist");

    logger.add(new LoggedFalcon("WristLeader", logger, wristLeaderMotor, WristLogging.Motor));
    logger.add(new LoggedFalcon("WristFollower", logger, wristFollowerMotor, WristLogging.Motor));

    logger.addBoolean("WristAtSetpoint", () -> isAtSetpoint(), WristLogging.Main);

    logger.addDouble("WristPosition", () -> ElevatorConstants.elevatorRotationsToMeters(getAngle().getDegrees()),
        WristLogging.Main);
    logger.addDouble("WristVelocity",
        () -> ElevatorConstants.elevatorRotationsToMeters(wristLeaderMotor.getVelocity().getValue()),
        WristLogging.Main);


  }
}
