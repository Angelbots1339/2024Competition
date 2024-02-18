// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Leds;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class ScoreAmp extends Command {

  private Elevator elevator;
  private Wrist wrist;
  private Indexer indexer;
  private Swerve swerve;

  private Timer timer = new Timer();

  /** Creates a new ScoreAmp. */
  public ScoreAmp(Elevator elevator, Wrist wrist, Indexer indexer, Swerve swerve) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.indexer = indexer;
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist, indexer, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.getInstance().scoringAmp = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.toHeight(ScoringConstants.ScoreAmp.height);
    wrist.toAngle(ScoringConstants.ScoreAmp.angle);

    if (elevator.isAtSetpoint() && wrist.isAtSetpoint()) {
      timer.start();

      if (timer.get() > 0.5) {
        indexer.runIndexerTorqueControl(ScoringConstants.ampScoringCurrentIndexer);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().scoringAmp = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
