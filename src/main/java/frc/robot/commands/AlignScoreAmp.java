// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class AlignScoreAmp extends Command {


  Elevator elevator;
  Wrist wrist;
  Indexer indexer;
  Swerve swerve;

  /** Creates a new AlignScoreAmp. */
  public AlignScoreAmp(Elevator elevator, Wrist wrist, Indexer indexer, Swerve swerve) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.indexer = indexer;
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist, indexer, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.toAngle(ScoringConstants.ScoreAmp.angle);
    elevator.toHeight(ScoringConstants.ScoreAmp.height);


    swerve.pidToPose(new Pose2d(FieldUtil.getAllianceAmpPosition(), Rotation2d.fromDegrees(90)));


    if(wrist.isAtSetpoint() && elevator.isAtSetpoint() && swerve.isAtPose()) {
      indexer.runIndexerDutyCycle(ScoringConstants.indexerScoringPercent);
    } else {
      indexer.disable();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.home();
    elevator.home();
    indexer.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
