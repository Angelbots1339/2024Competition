// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.tuning.GlobalVoltageTuning;
import frc.lib.util.tuning.ShooterTuning;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private TuningMode tuningMode = TuningMode.DISABLED;
  
  /***** Instancing Subsystems *****/
  private final Swerve swerve = Constants.GeneratedSwerveConstants.Swerve;
  // private final Intake intake = new Intake();
  // private final Shooter shooter = new Shooter();
  // private final Wrist wrist = new Wrist();
  // private final Elevator elevator = new Elevator();
  // private final Indexer indexer = new Indexer();





  /***** Driver Controls *****/
  private final XboxController driveController = new XboxController(0);

  private Supplier<Double> translationX = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftY(), true);
  private Supplier<Double> translationY = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftX(), true);
  private Supplier<Double> rotation = () -> DriverConstants.fixRotationJoystickValues(-driveController.getRightX(),
      false);

  private Trigger runIntake = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
  private Trigger runOuttake = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);

  private Trigger shoot = new JoystickButton(driveController, XboxController.Button.kA.value);
  private Trigger scoreAmp = new JoystickButton(driveController, XboxController.Button.kB.value);

  private Trigger resetGyro = new JoystickButton(driveController, XboxController.Button.kStart.value);

  
  private void configDriverBindings() {
    resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    // runIntake.whileTrue(new StartEndCommand(() -> intake.runIntakeDutyCycle(0.75), () -> intake.runIntakeDutyCycle(0), intake));
    // runOuttake.whileTrue(new StartEndCommand(() -> intake.runIntakeDutyCycle(-0.75), () -> intake.runIntakeDutyCycle(0), intake));

    // runIntake.whileTrue(new IntakeNote(intake, indexer, wrist, elevator, () -> false));


  }

  private void configOperatorBindings() {

  }




  /***** Initialization *****/
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configDefaultCommands();
    configDriverBindings();
    configOperatorBindings();
    
  }

  private void configDefaultCommands() {
    swerve.setDefaultCommand(swerve.drive(translationX, translationY, rotation, () -> true, () -> true));
    // wrist.setDefaultCommand(new RunCommand(wrist::home, wrist));
    // elevator.setDefaultCommand(new RunCommand(elevator::home, elevator));
    // intake.setDefaultCommand(new RunCommand(() -> intake.disable(), intake));
    // indexer.setDefaultCommand(new RunCommand(() -> indexer.disable(), indexer));
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.disable(), shooter));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }





  /***** Tuning *****/

  /**
   * Call at the start of test mode to initialize tuning
   */
  public void tuningInitialization() {

    // if (tuningMode == TuningMode.VOLTAGE) {
    //   GlobalVoltageTuning.initialize(elevator, wrist, shooter, intake, indexer);
    // } else if (tuningMode == TuningMode.SHOOTER) {
    //   ShooterTuning.initialize(shooter);
    // }
  }

  /**
   * Call periodically to run  tuning
   */
  public void tuningPeriodic() {
    if (tuningMode == TuningMode.VOLTAGE) {
      GlobalVoltageTuning.periodic();
    } else if (tuningMode == TuningMode.SHOOTER) {
      ShooterTuning.periodic(driveController);
    }
  }

  /**
   * Call at the end of test mode to end tuning
   */
  public void tuningEnd() {
    if (tuningMode == TuningMode.VOLTAGE) {
      GlobalVoltageTuning.end();
    } else if (tuningMode == TuningMode.SHOOTER) {
      ShooterTuning.end();
    }
  }


  private enum TuningMode {
    DISABLED, VOLTAGE, SHOOTER
  }
}
