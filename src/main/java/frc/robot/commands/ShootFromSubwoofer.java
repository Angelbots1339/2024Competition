// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ShootFromSubwoofer extends Command {

  private Elevator elevator;
  private Wrist wrist;
  private Shooter shooter;
  private Indexer indexer;

  private Timer shootTimer = new Timer();

  /** Creates a new ShootFromSpeaker. */
  public ShootFromSubwoofer(Elevator elevator, Wrist wrist, Shooter shooter, Indexer indexer) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.shooter = shooter;
    this.indexer = indexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist, shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.toAngle(ScoringConstants.SubwooferShot.angle);
    elevator.toHeight(ScoringConstants.SubwooferShot.height);
    shooter.shooterToRMP(ScoringConstants.shooterSetpointClose[0], ScoringConstants.shooterSetpointClose[1]);

    shootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(wrist.isAtSetpoint() && elevator.isAtSetpoint() && shooter.isAtSetpoint() && shootTimer.get() > ScoringConstants.shootingWaitTime){
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
    shooter.disable();
    indexer.disable();

    shootTimer.stop();
    shootTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
