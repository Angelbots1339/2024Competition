// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class GlobalVoltageTuning {

  private static Elevator elevator;
  private static Wrist wrist;
  private static Shooter shooter;
  private static Intake intake;
  private static Indexer indexer;

  private static GenericEntry elevatorVolts;
  private static GenericEntry wristVolts;
  private static GenericEntry shooterVolts;
  private static GenericEntry intakeVolts;
  private static GenericEntry indexerVolts;

  private static boolean hasBeenInitialized = false;

  private static XboxController testController;

  public static void initialize(Elevator elevatorInstance, Wrist wristInstance, Shooter shooterInstance,
      Intake intakeInstance, Indexer indexerInstance) {

    if (!hasBeenInitialized) {
      testController = new XboxController(2);
      elevatorVolts = Shuffleboard.getTab("Tuning").add("Elevator Volts", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -12, "max", 12))
          .getEntry();

      wristVolts = Shuffleboard.getTab("Tuning").add("Wrist Volts", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -12, "max", 12))
          .getEntry();

      shooterVolts = Shuffleboard.getTab("Tuning").add("Shooter Volts", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -12, "max", 12))
          .getEntry();

      intakeVolts = Shuffleboard.getTab("Tuning").add("Intake Volts", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -12, "max", 12))
          .getEntry();

      indexerVolts = Shuffleboard.getTab("Tuning").add("Indexer Volts", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -12, "max", 12))
          .getEntry();

      elevator = elevatorInstance;
      wrist = wristInstance;
      shooter = shooterInstance;
      intake = intakeInstance;
      indexer = indexerInstance;

      hasBeenInitialized = true;
    }
  }

  public static void periodic() {

    elevator.setVoltage(elevatorVolts.getDouble(0));
    wrist.setVoltage(wristVolts.getDouble(0));
    shooter.setVoltage(shooterVolts.getDouble(0));
    intake.setVoltage(intakeVolts.getDouble(0));
    indexer.setVoltage(indexerVolts.getDouble(0));

  }

  public static void end() {
    elevator.disable();
    wrist.disable();
    shooter.disable();
    intake.disable();
    indexer.disable();

  }

}
