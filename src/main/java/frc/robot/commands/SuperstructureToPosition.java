// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.WristElevatorState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SuperstructureToPosition extends Command {

  Elevator elevator;
  Wrist wrist;
  Supplier<WristElevatorState> state;

  /** Creates a new SuperstructureToPosition. */
  public SuperstructureToPosition(Elevator elevator, Wrist wrist, Supplier<WristElevatorState> state) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.state = state;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.toHeight(state.get().height);
    wrist.toAngle(state.get().angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.toHeight(state.get().height);
    wrist.toAngle(state.get().angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.home();
    wrist.home();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint() && wrist.isAtSetpoint();
  }
}
