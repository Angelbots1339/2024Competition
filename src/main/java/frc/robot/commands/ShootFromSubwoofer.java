// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Leds;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class ShootFromSubwoofer extends Command {

  private Elevator elevator;
  private Wrist wrist;
  private Shooter shooter;
  private Swerve swerve;
  private Indexer indexer;

  private Supplier<Double> translationX;
  private Supplier<Double> translationY;
  private Supplier<Boolean> actuallyShoot;

  // private Timer shootTimer = new Timer();

  /** Creates a new ShootFromSpeaker. */
  public ShootFromSubwoofer(Elevator elevator, Wrist wrist, Shooter shooter, Swerve swerve, Indexer indexer,
      Supplier<Double> translationX, Supplier<Double> translationY, Supplier<Boolean> actuallyShoot) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.shooter = shooter;
    this.indexer = indexer;
    this.swerve = swerve;

    this.translationX = translationX;
    this.translationY = translationY;
    this.actuallyShoot = actuallyShoot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist, shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.toAngle(ScoringConstants.SubwooferShot.angle);
    elevator.toHeight(ScoringConstants.SubwooferShot.height);
    shooter.shooterToRMP(ScoringConstants.shooterSetpointClose[0], ScoringConstants.shooterSetpointClose[1]);

    Leds.getInstance().shooting = true;

    // shootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    wrist.toAngle(ScoringConstants.SubwooferShot.angle);
    elevator.toHeight(ScoringConstants.SubwooferShot.height);
    shooter.shooterToRMP(ScoringConstants.shooterSetpointClose[0], ScoringConstants.shooterSetpointClose[1]);

    swerve.angularDriveRequest(() -> translationX.get(),
        () -> translationY.get(),
        () -> Rotation2d.fromDegrees(0), () -> true);

    if (actuallyShoot.get()) {
      indexer.setVoltage(ScoringConstants.indexerScoringVoltage);
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

    Leds.getInstance().shooting = false;

    // shootTimer.stop();
    // shootTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
