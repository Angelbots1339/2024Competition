// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.lib.util.Leds;
import frc.lib.util.PoseEstimation;
import frc.robot.Constants.IndexerConstants;
import frc.robot.regressions.SpeakerShotRegression;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class Shoot extends Command {

  Shooter shooter;
  Wrist wrist;
  Elevator elevator;
  Swerve swerve;
  Indexer indexer;

  Supplier<Double> translationX;
  Supplier<Double> translationY;
  Supplier<Boolean> overrideIndexerSensor;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Wrist wrist, Elevator elevator, Swerve swerve, Indexer indexer,
      Supplier<Double> translationX, Supplier<Double> translationY, Supplier<Boolean> overrideIndexerSensor) {

    this.shooter = shooter;
    this.wrist = wrist;
    this.elevator = elevator;
    this.swerve = swerve;
    this.indexer = indexer;

    this.translationX = translationX;
    this.translationY = translationY;
    this.overrideIndexerSensor = overrideIndexerSensor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, wrist, elevator, swerve, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.getInstance().shooting = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d virtualTarget = PoseEstimation.calculateVirtualSpeakerOffset(FieldUtil.getAllianceSpeakerPosition());

    double targetDistance = PoseEstimation.getEstimatedPose().getTranslation()
        .getDistance(virtualTarget);

    Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the angle to turn the robot to
        Math.atan((PoseEstimation.getEstimatedPose().getX() - virtualTarget.getX())
            / (PoseEstimation.getEstimatedPose().getY() - virtualTarget.getY())))
        .plus(Rotation2d.fromRadians(Math.PI));

    wrist.wristToPosition(SpeakerShotRegression.wristRegression.predict(targetDistance));
    elevator.home();
    shooter.shooterToRMP(SpeakerShotRegression.flywheelRegression.predict(targetDistance));

    swerve.angularDrive(translationX, translationY, robotAngle, () -> true, () -> true);

    if ((indexer.isNotePresent() || overrideIndexerSensor.get()) && shooter.isAtSetpoint() && wrist.isAtSetpoint()
        && elevator.isAtSetpoint() && swerve.isAtAngularDriveSetpoint()) {
      indexer.runIndexerTorqueControl(IndexerConstants.indexingTargetCurrent);
    } else {
      indexer.disable();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disable();
    indexer.disable();
    wrist.home();
    elevator.home();

    Leds.getInstance().shooting = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
