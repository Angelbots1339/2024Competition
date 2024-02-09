// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CompletableFuture;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.Leds;
import frc.lib.util.logging.Logger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private static final double lowBatteryVoltage = 10.0;
  private static final double lowBatteryDisabledTime = 1.5;

  private final Timer disabledTimer = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    DataLogManager.start();

    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    CompletableFuture.runAsync(() -> {
      Logger.getInstance().log(0);
    });

    // Update low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
     if (RobotController.getBatteryVoltage() < lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      Leds.getInstance().lowBatteryAlert = true;
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.tuningInitialization();
  }

  @Override
  public void testPeriodic() {
    m_robotContainer.tuningPeriodic();
  }

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.tuningEnd();
  }

  @Override
  public void simulationPeriodic() {
  }

}
