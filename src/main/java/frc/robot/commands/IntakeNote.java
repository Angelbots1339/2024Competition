// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperstructureStates;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class IntakeNote extends Command {

  private Intake intake;
  private Indexer indexer;
  private Wrist wrist;
  private Elevator elevator;
  private Supplier<Boolean> endWhenNoteDetected;

  private boolean noteDetected = false;

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake, Indexer indexer, Wrist wrist, Elevator elevator,
      Supplier<Boolean> endWhenNoteDetected) {

    this.intake = intake;
    this.indexer = indexer;
    this.wrist = wrist;
    this.elevator = elevator;
    this.endWhenNoteDetected = endWhenNoteDetected;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, indexer, wrist, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runIntakeTorqueControl(IntakeConstants.intakingTargetCurrent);
    indexer.runIndexerTorqueControl(IndexerConstants.indexingTargetCurrent);
    wrist.wristToPosition(SuperstructureStates.Handoff.angle);
    elevator.toHeight(SuperstructureStates.Handoff.height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.isNotePresent() || indexer.isNotePresent()) {
      noteDetected = true;
    }

    wrist.wristToPosition(SuperstructureStates.Handoff.angle);
    elevator.toHeight(SuperstructureStates.Handoff.height);

    if (!intake.isNotePresent() && !noteDetected) {
      intake.runIntakeTorqueControl(IntakeConstants.intakingTargetCurrent);
    } else if(noteDetected && !wrist.isAtSetpoint() && !elevator.isAtSetpoint()) {
      intake.disable();
    } else if(noteDetected && wrist.isAtSetpoint() && elevator.isAtSetpoint()){
      intake.runIntakeTorqueControl(IntakeConstants.intakingTargetCurrent);
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

    if (!indexer.isNotePresent() && intake.isNotePresent()) {
      CommandScheduler.getInstance().schedule(new HandoffNote(intake, indexer, wrist, elevator));
    } else {
      intake.disable();
      indexer.disable();
      wrist.disable();
      elevator.disable();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endWhenNoteDetected.get() && indexer.isNotePresent() && !intake.isNotePresent()) {
      return true;
    }
    return false;
  }
}
