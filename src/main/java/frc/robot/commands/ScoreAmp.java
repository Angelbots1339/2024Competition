// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
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

  Supplier<Double> translationX;
  Supplier<Double> translationY;



  /** Creates a new ScoreAmp. */
  public ScoreAmp(Elevator elevator, Wrist wrist, Indexer indexer, Swerve swerve, Supplier<Double> translationX,
      Supplier<Double> translationY) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.swerve = swerve;
    this.indexer = indexer;

    this.translationX = translationX;
    this.translationY = translationY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist, swerve);
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

    if (ScoringConstants.ScoreAmp.angle.getDegrees() > 90 || elevator.getLeaderPosition() > 0.15) {

      wrist.toAngle(ScoringConstants.ScoreAmp.angle);
    }

    swerve.angularDriveRequest(() -> translationX.get() * 1,
        () -> translationY.get() * 1,
        () -> Rotation2d.fromDegrees(FieldUtil.isAllianceBlue() ? 270 : 90),
        () -> true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().scoringAmp = false;

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
