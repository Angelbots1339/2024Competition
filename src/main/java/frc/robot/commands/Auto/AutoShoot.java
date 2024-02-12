// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.lib.util.Leds;
import frc.lib.util.PoseEstimation;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.regressions.SpeakerShotRegression;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class AutoShoot extends Command {

  private Shooter shooter;
  private Wrist wrist;
  private Elevator elevator;
  private Swerve swerve;
  private Indexer indexer;

  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter, Wrist wrist, Elevator elevator, Swerve swerve, Indexer indexer) {
    this.shooter = shooter;
    this.wrist = wrist;
    this.elevator = elevator;
    this.swerve = swerve;
    this.indexer = indexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, wrist, elevator, swerve, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the angle to turn the robot to
        Math.atan((PoseEstimation.getEstimatedPose().getX()
            - PoseEstimation.calculateVirtualSpeakerOffset(FieldUtil.getAllianceSpeakerPosition()).getX())
            / (PoseEstimation.getEstimatedPose().getY()
                - PoseEstimation.calculateVirtualSpeakerOffset(FieldUtil.getAllianceSpeakerPosition()).getY())))
        .plus(Rotation2d.fromRadians(Math.PI));

    swerve.setAutoOverrideRotation(true, robotAngle);

    Leds.getInstance().shooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Translation2d virtualTarget = PoseEstimation.calculateVirtualSpeakerOffset(FieldUtil.getAllianceSpeakerPosition());

    double targetDistance = PoseEstimation.getEstimatedPose().getTranslation()
        .getDistance(virtualTarget);

    wrist.toAngle(SpeakerShotRegression.wristRegression.predict(targetDistance));
    elevator.home();
    shooter.shooterToRMP(SpeakerShotRegression.flywheelRegression.predict(targetDistance));

    if (indexer.isNotePresent() && shooter.isAtSetpoint() && wrist.isAtSetpoint()
        && elevator.isAtSetpoint() && swerve.isAtAngularDriveSetpoint()) {
      indexer.runIndexerTorqueControl(ScoringConstants.indexingTargetCurrent);
    } else if (!indexer.isNotePresent()) {
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

    swerve.setAutoOverrideRotation(false);

    Leds.getInstance().shooting = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
