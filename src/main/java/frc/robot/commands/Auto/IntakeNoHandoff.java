// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Leds;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Intake;

public class IntakeNoHandoff extends Command {

  private Intake intake;

  /** Creates a new IntakeNoHandoff. */
  public IntakeNoHandoff(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Leds.getInstance().intaking = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.isNotePresent()) {
      intake.disable();;
    } else {
      intake.runIntakeTorqueControl(ScoringConstants.intakingTargetCurrent);
    }

    // Leds.getInstance().hasGamePiece = intake.isNotePresent();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disable();
    // Leds.getInstance().intaking = false;
    // Leds.getInstance().hasGamePiece = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
