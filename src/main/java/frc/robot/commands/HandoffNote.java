// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperstructureStates;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class HandoffNote extends Command {

    private Intake intake;
  private Indexer indexer;
  private Wrist wrist;
  private Elevator elevator;

  /** Creates a new HandoffNote. */
  public HandoffNote(Intake intake, Indexer indexer, Wrist wrist, Elevator elevator) {

    this.intake = intake;
    this.indexer = indexer;
    this.wrist = wrist;
    this.elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, indexer, wrist, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    wrist.wristToPosition(SuperstructureStates.Handoff.angle);
    elevator.toHeight(SuperstructureStates.Handoff.height);

 
    if(wrist.isAtSetpoint() && elevator.isAtSetpoint()) {
      intake.runIntakeTorqueControl(IntakeConstants.intakingTargetCurrent);
    } else {
      intake.disable();
    }

    if (!indexer.isNotePresent()) {
      indexer.runIndexerTorqueControl(IndexerConstants.indexingTargetCurrent);
    } else {
      indexer.disable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disable();
    indexer.disable();
    elevator.home();
    wrist.home();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (indexer.isNotePresent() && !intake.isNotePresent()) {
      return true;
    }
    return false;
  }
}
