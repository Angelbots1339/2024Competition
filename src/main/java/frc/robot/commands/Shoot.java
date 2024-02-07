// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.lib.util.PoseEstimation;
import frc.robot.regressions.SpeakerShotRegression;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class Shoot extends Command {

  Shooter shooter;
  Wrist wrist;
  Swerve swerve;

  Supplier<Double> translationX;
  Supplier<Double> translationY;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Wrist wrist, Swerve swerve, Supplier<Double> translationX, Supplier<Double> translationY) {

    this.shooter = shooter;
    this.wrist = wrist;
    this.swerve = swerve;

    this.translationX = translationX;
    this.translationY = translationY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, wrist, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetDistance = PoseEstimation.getEstimatedPose().getTranslation()
        .getDistance(PoseEstimation.calculateVirtualSpeakerOffset(FieldUtil.getAllianceSpeakerPosition()));

    Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the angle to turn the robot to
          Math.atan((PoseEstimation.getEstimatedPose().getX() - FieldUtil.getAllianceSpeakerPosition().getX())
            / (PoseEstimation.getEstimatedPose().getY() - FieldUtil.getAllianceSpeakerPosition().getY()))); 

    wrist.wristToPosition(SpeakerShotRegression.wristRegression.predict(targetDistance));
    shooter.shooterToRMP(SpeakerShotRegression.flywheelRegression.predict(targetDistance));

    swerve.angularDrive(translationX, translationY, robotAngle, () -> true, () -> true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disable();
    wrist.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
