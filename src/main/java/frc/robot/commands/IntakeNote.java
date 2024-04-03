// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Leds;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class IntakeNote extends Command {

  private Intake intake;
  private Indexer indexer;
  private Wrist wrist;
  private Elevator elevator;

  private boolean noteDetected = false;

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake, Indexer indexer, Wrist wrist, Elevator elevator) {

    this.intake = intake;
    this.indexer = indexer;
    this.wrist = wrist;
    this.elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, indexer, wrist, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // intake.runIntakeTorqueControl(ScoringConstants.intakingTargetCurrent);
    // indexer.runIndexerTorqueControl(ScoringConstants.indexingTargetCurrent);
    wrist.toAngle(ScoringConstants.Handoff.angle);
    elevator.toHeight(ScoringConstants.Handoff.height);

    Leds.getInstance().intaking = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.isNotePresent() || indexer.isNotePresent()) {
      noteDetected = true;
      Leds.getInstance().hasGamePiece = noteDetected;
    }

    wrist.toAngle(ScoringConstants.Handoff.angle);
    elevator.toHeight(ScoringConstants.Handoff.height);

    if (indexer.isNotePresent()) {
      intake.disable();

    } else if (!intake.isNotePresent() && !indexer.isNotePresent()) {
      intake.setVoltage(ScoringConstants.intakingTargetVoltage);

    } else if (intake.isNotePresent() && !wrist.isAtSetpoint() && !elevator.isAtSetpoint()) {
      intake.disable();

    } else if (wrist.isAtSetpoint() && elevator.isAtSetpoint()) {
      intake.setVoltage(ScoringConstants.intakingTargetVoltage);
    }

    indexer.indexNoteToTarget();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // if (indexer.isNotePresent() && !intake.isNotePresent()) {
    intake.disable();
    indexer.disable();
    wrist.home();
    elevator.home();
    // }

    // System.out.println("Intake Command Ended");

    Leds.getInstance().intaking = false;
    Leds.getInstance().hasGamePiece = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (indexer.isNoteAtTarget()) {
      return true;
    }
    return false;
  }
}
