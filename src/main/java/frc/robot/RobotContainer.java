// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.GeneratedSwerveConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveController = new CommandXboxController(0); // My joystick

  private final Swerve swerve = Constants.GeneratedSwerveConstants.Swerve;
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  // private final Wrist wrist = new Wrist();
  // private final Elevator elevator = new Elevator();

  private Supplier<Double> translationX = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftY(), true);
  private Supplier<Double> translationY = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftX(), true);
  private Supplier<Double> rotation = () -> DriverConstants.fixRotationJoystickValues(-driveController.getRightX(),
      false);

  private GenericEntry TopSpeed = Shuffleboard.getTab("Testing").add("Top Wheel Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 6000))
      .getEntry();
  private GenericEntry BottomSpeed = Shuffleboard.getTab("Testing").add("Bottom Wheel Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 6000))
      .getEntry();

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configDefaultCommands();
    configureBindings();

    if (Utils.isSimulation()) {
      swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  private void configDriverBindings() {
    driveController.start().onTrue(swerve.runOnce(swerve::seedFieldRelative));

    driveController.leftBumper().whileTrue(new StartEndCommand(() -> intake.runIntakeDutyCycle(0.3), intake::disable, intake));
    driveController.rightBumper().whileTrue(new StartEndCommand(() -> intake.runIntakeDutyCycle(-0.3), intake::disable, intake));

    driveController.a().whileTrue(new StartEndCommand(
        () -> shooter.shooterToRMP(TopSpeed.getDouble(0), BottomSpeed.getDouble(0)), shooter::disable, shooter));

  }

  private void configOperatorBindings() {

  }

  private void configureBindings() {

    configDriverBindings();
    configOperatorBindings();
  }

  private void configDefaultCommands() {
    swerve.setDefaultCommand( // Drivetrain will execute this command periodically
        swerve.drive(translationX, translationY, rotation, () -> true, () -> true));
    // wrist.setDefaultCommand(new RunCommand(wrist::home, wrist));
    // elevator.setDefaultCommand(new RunCommand(elevator::home, elevator));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
