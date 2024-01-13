// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

  private SendableChooser autoChooser = new SendableChooser<>();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveController = new CommandXboxController(0); // My joystick

  private final Swerve swerve = Constants.SwerveConstants.Swerve; // My drivetrain
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final Telemetry logger = new Telemetry(SwerveConstants.maxSpeed);

  private Supplier<Double> translationX = () -> MathUtil.applyDeadband(Math.pow(-driveController.getLeftY(), 2), 0.1) * SwerveConstants.maxSpeed;
  private Supplier<Double> translationY = () -> MathUtil.applyDeadband(Math.pow(-driveController.getLeftX(), 2), 0.1) * SwerveConstants.maxSpeed;
  private Supplier<Double> rotation = () -> MathUtil.applyDeadband(-driveController.getRightX(), 0.1) * SwerveConstants.maxAngularRate;


   public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand( // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> driveRequest.withVelocityX(translationX.get()) // Drive forward with negative Y (forward)
            .withVelocityY(translationY.get()) // Drive left with negative X (left)
            .withRotationalRate(rotation.get()) // Drive counterclockwise with negative X (left)
        ));

    // reset the field-centric heading on left bumper press
    driveController.start().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));

    if (Utils.isSimulation()) {
      swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    swerve.registerTelemetry(logger::telemeterize);
  }

 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
