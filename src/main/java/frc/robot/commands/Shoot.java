// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class Shoot extends Command {

  private Shooter shooter;
  private Wrist wrist;
  private Elevator elevator;
  private Swerve swerve;
  private Indexer indexer;

  private Supplier<Double> translationX;
  private Supplier<Double> translationY;
  private Supplier<Boolean> overrideSetpoints;



  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Wrist wrist, Elevator elevator, Swerve swerve, Indexer indexer,
      Supplier<Double> translationX, Supplier<Double> translationY, Supplier<Boolean> overrideSetpoints) {

    this.shooter = shooter;
    this.wrist = wrist;
    this.elevator = elevator;
    this.swerve = swerve;
    this.indexer = indexer;

    this.translationX = translationX;
    this.translationY = translationY;
    this.overrideSetpoints = overrideSetpoints;

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

    // Translation2d target = ScoringConstants.shootWhileMoving
    // ?
    // PoseEstimation.calculateVirtualSpeakerOffset(FieldUtil.getAllianceSpeakerPosition())
    // : FieldUtil.getAllianceSpeakerPosition();
    Translation2d target = FieldUtil.getAllianceSpeakerPosition();

    double targetDistance = PoseEstimation.getEstimatedPose().getTranslation()
        .getDistance(target);

    
  

    Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians(  // Find the angle to turn the robot to
    Math.atan((PoseEstimation.getEstimatedPose().getY() - target.getY())
        / (PoseEstimation.getEstimatedPose().getX() - target.getX())));

    wrist.toAngle(SpeakerShotRegression.calculateWristAngle(targetDistance));
    elevator.home();

    // double[] speeds = targetDistance < ScoringConstants.flywheelDistanceCutoff ? ScoringConstants.shooterSetpointClose
    //     : ScoringConstants.shooterSetpointFar;
    shooter.shooterToRMP(ScoringConstants.shooterSetpointFar[0], ScoringConstants.shooterSetpointFar[1]);

    swerve.angularDriveRequest(() -> translationX.get() *
        ScoringConstants.shootingDriveScalar,
        () -> translationY.get() * ScoringConstants.shootingDriveScalar, robotAngle,
        () -> true);

 
    if ((shooter.isAtSetpoint() && wrist.isAtSetpoint() && swerve.isAtAngularDriveSetpoint()) || overrideSetpoints.get()) {
      indexer.runIndexerDutyCycle(ScoringConstants.indexingTargetPercent);
    } else {
      indexer.indexNoteToTarget();
    }

    SmartDashboard.putNumber("TargetAngle", SpeakerShotRegression.calculateWristAngle(targetDistance).getDegrees());
    SmartDashboard.putNumber("TargetDistance", targetDistance);

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
