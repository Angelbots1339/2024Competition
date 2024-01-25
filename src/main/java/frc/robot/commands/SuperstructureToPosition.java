// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.WristElevatorState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SuperstructureToPosition extends Command {

  Elevator elevator;
  Wrist wrist;
  WristElevatorState state;

  /** Creates a new SuperstructureToPosition. */
  public SuperstructureToPosition(Elevator elevator, Wrist wrist, WristElevatorState state) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.state = state;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.toHeight(state.height);
    wrist.wristToPosition(state.angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
