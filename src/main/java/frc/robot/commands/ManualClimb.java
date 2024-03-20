// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Leds;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class ManualClimb extends Command {

  private Elevator elevator;
  private Wrist wrist;
  private Supplier<Double> extend;
  private Supplier<Double> retract;
  private Supplier<Double> wristRotation;

  /** Creates a new ManualClimb. */
  public ManualClimb(Elevator elevator, Wrist wrist, Supplier<Double> extend, Supplier<Double> retract,
      Supplier<Double> wristRotation) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.extend = extend;
    this.retract = retract;
    this.wristRotation = wristRotation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.getInstance().climbing = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(extend.get() - retract.get()) > 0.1) {
      elevator.setVoltage((extend.get() - retract.get()) * 10);
    } else {
      elevator.holdPosition();
    }

    // if (Math.abs(wristRotation.get()) > 0.1) {
    //   wrist.setVoltage(wristRotation.get() * 6);
    // } else {
    //   wrist.holdPosition();
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.home();
    wrist.home();
    Leds.getInstance().climbing = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
