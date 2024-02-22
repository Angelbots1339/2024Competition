// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.Leds;
import frc.lib.util.tuning.GlobalVoltageTuning;
import frc.lib.util.tuning.ShooterTuning;
import frc.lib.util.tuning.SuperstructureTuning;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.SuperstructureToPosition;
import frc.robot.commands.Auto.AutoShoot;
import frc.robot.commands.Auto.IntakeNoHandoff;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private TuningMode tuningMode = TuningMode.VOLTAGE;

  /***** Instancing Subsystems *****/
  private final Swerve swerve = Constants.GeneratedSwerveConstants.Swerve;
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator();
  private final Indexer indexer = new Indexer();

  private Timer shootTimer = new Timer();

  /***** Driver Controls *****/
  private final XboxController driveController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private Supplier<Double> translationX = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftY(), true);
  private Supplier<Double> translationY = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftX(), true);
  private Supplier<Double> rotation = () -> DriverConstants.fixRotationJoystickValues(-driveController.getRightX(),
      false);

  private boolean climbMode = false;

  private Trigger runIntake = new Trigger(() -> driveController.getLeftBumper() && !climbMode);
  private Trigger runOuttake = new Trigger(() -> driveController.getRightBumper() && !climbMode);

  private Trigger shoot = new Trigger(() -> driveController.getAButton() && !climbMode);
  private Trigger scoreAmp = new Trigger(() -> driveController.getBButton() && !climbMode);

  private Trigger resetGyro = new Trigger(() -> driveController.getStartButton());
  private Trigger toggleClimbMode = new Trigger(() -> driveController.getBackButton());

  private Trigger extendClimb = new Trigger(() -> driveController.getLeftTriggerAxis() > 0.2 && climbMode);
  private Trigger retractClimb = new Trigger(() -> driveController.getRightTriggerAxis() > 0.2 && climbMode);

  private void configDriverBindings() {
    resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    toggleClimbMode.onTrue(new InstantCommand(() -> {
      climbMode = !climbMode;
      // Leds.getInstance().climbing = climbMode;
    }));
    // runIntake.whileTrue(new StartEndCommand(() ->
    // intake.runIntakeDutyCycle(0.1), () -> intake.runIntakeDutyCycle(0),
    // intake));
    // runOuttake.whileTrue(new StartEndCommand(() ->
    // intake.runIntakeDutyCycle(-0.1), () -> intake.runIntakeDutyCycle(0),
    // intake));

    // runIntake.whileTrue(new RunCommand(() -> {
    //   intake.runIntakeDutyCycle(0.4);
    //   indexer.runIndexerDutyCycle(0.3);
    // },
    //     intake));

    runOuttake.whileTrue(new RunCommand(() -> {
      intake.runIntakeDutyCycle(-0.4);
      indexer.runIndexerDutyCycle(-0.4);
    },
        intake));

    // shoot.whileTrue(new StartEndCommand(() -> wrist.toAngle(0.5), () ->
    // wrist.disable(), wrist));
    // scoreAmp.whileTrue(new StartEndCommand(() -> wrist.toAngle(0), () ->
    // wrist.disable(), wrist));
    // shoot.whileTrue(new StartEndCommand(() -> elevator.toHeight(0.45), () ->
    // elevator.disable(), elevator));
    // scoreAmp.whileTrue(new StartEndCommand(() -> elevator.toHeight(0), () ->
    // elevator.disable(), elevator));


    // shoot.whileTrue(new SuperstructureToPosition(elevator, wrist, () -> ScoringConstants.Handoff));
    scoreAmp.whileTrue(new SuperstructureToPosition(elevator, wrist, () -> ScoringConstants.ScoreAmp));
    
    shoot.whileTrue(new InstantCommand(() -> {

      wrist.toAngle(Rotation2d.fromDegrees(145));
      elevator.home();
      shooter.shooterToRMP(6000, 5000);

      shootTimer.start();

    }).andThen(new RunCommand(() -> {

      if(wrist.isAtSetpoint() && elevator.isAtSetpoint() && shooter.isAtSetpoint() && shootTimer.get() > 0.1){
        indexer.runIndexerDutyCycle(ScoringConstants.indexingTargetPercent);
      } else {
        indexer.disable();
      }

    }, elevator, wrist, indexer, shooter)).finallyDo(() -> {
      shootTimer.stop();
      shootTimer.reset();
    }));

    // shoot.whileTrue(new StartEndCommand(() -> shooter.setVoltage(4), () -> shooter.disable(), shooter));
    



    extendClimb.whileTrue(new StartEndCommand(() -> elevator.setVoltage(driveController.getLeftTriggerAxis() * 8),
        () -> elevator.disable(), elevator));
    retractClimb.whileTrue(new StartEndCommand(() -> elevator.setVoltage(-driveController.getRightTriggerAxis() * 8),
        () -> elevator.disable(), elevator));

    runIntake.whileTrue(new IntakeNote(intake, indexer, wrist, elevator, () ->
    false));

    // runOuttake.whileTrue(new StartEndCommand(() ->
    // intake.runIntakeTorqueControl(-ScoringConstants.intakingTargetCurrent), () ->
    // intake.disable(),
    // intake));

    // shoot.whileTrue(new Shoot(shooter, wrist, elevator, swerve, indexer,
    // translationX, translationY, () -> false));

  }

  private void configOperatorBindings() {

  }

  /***** Initialization *****/
  public RobotContainer() {

    // NamedCommands.registerCommand("shoot", new AutoShoot(shooter, wrist,
    //     elevator, swerve, indexer));
    // NamedCommands.registerCommand("intakeNote", new IntakeNote(intake, indexer,
    //     wrist, elevator, () -> false));
    // NamedCommands.registerCommand("runIntakeNoHandoff", new IntakeNoHandoff(intake));

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configDefaultCommands();
    configDriverBindings();
    configOperatorBindings();
    initializeEndgameAlerts();

  }

  private void configDefaultCommands() {
    swerve.setDefaultCommand(swerve.drive(translationX, translationY, rotation, () -> true, () -> true));
    wrist.setDefaultCommand(new InstantCommand(wrist::home, wrist));
    elevator.setDefaultCommand(new InstantCommand(() -> {
      // if (climbMode) {
      //   elevator.holdPosition();
      // } else {
        elevator.home();
      // }
    }, elevator));
    // intake.setDefaultCommand(new InstantCommand(() -> intake.disable(), intake));
    // indexer.setDefaultCommand(new InstantCommand(() -> indexer.disable(), indexer));
    shooter.setDefaultCommand(new InstantCommand(() -> shooter.disable(), shooter));
  }

  private void initializeEndgameAlerts() {
    new Trigger(() -> {
      return DriverStation.isTeleopEnabled()
          && DriverStation.getMatchTime() > 0.0
          && DriverStation.getMatchTime() <= Math.round(DriverConstants.endgameAlert1);
    })
        .onTrue(
            Commands.run(
                () -> {
                  Leds.getInstance().endgameAlert = true;
                  driveController.setRumble(RumbleType.kBothRumble, 1.0);
                  operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                })
                .withTimeout(1.5)
                .andThen(
                    Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = false;
                          driveController.setRumble(RumbleType.kBothRumble, 0.0);
                          operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                        })
                        .withTimeout(1.0)));

    new Trigger(
        () -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0.0
            && DriverStation.getMatchTime() <= Math.round(DriverConstants.endgameAlert2))
        .onTrue(
            Commands.sequence(
                Commands.run(
                    () -> {
                      Leds.getInstance().endgameAlert = true;
                      driveController.setRumble(RumbleType.kBothRumble, 1.0);
                      operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                    })
                    .withTimeout(0.25),
                Commands.run(
                    () -> {
                      Leds.getInstance().endgameAlert = false;
                      driveController.setRumble(RumbleType.kBothRumble, 0.0);
                      operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                    })
                    .withTimeout(0.1),
                Commands.run(
                    () -> {
                      Leds.getInstance().endgameAlert = true;
                      driveController.setRumble(RumbleType.kBothRumble, 1.0);
                      operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                    })
                    .withTimeout(0.25),
                Commands.run(
                    () -> {
                      Leds.getInstance().endgameAlert = false;
                      driveController.setRumble(RumbleType.kBothRumble, 0.0);
                      operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                    })
                    .withTimeout(1.0)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /***** Tuning *****/

  /**
   * Call at the start of test mode to initialize tuning
   */
  public void tuningInitialization() {

    switch (tuningMode) {
      case DISABLED:
        break;
      case VOLTAGE:
        GlobalVoltageTuning.initialize(elevator, wrist, shooter, intake, indexer);
        break;
      case SHOOTER:
        ShooterTuning.initialize(shooter, indexer, wrist);
        break;
      case SUPERSTRUCTURE:
        SuperstructureTuning.initialize(elevator, wrist, indexer);
        break;
      default:
        break;
    }
  }

  /**
   * Call periodically to run tuning
   */
  public void tuningPeriodic() {
    switch (tuningMode) {
      case DISABLED:
        break;
      case VOLTAGE:
        GlobalVoltageTuning.periodic();
        break;
      case SHOOTER:
        ShooterTuning.periodic();
        break;
      case SUPERSTRUCTURE:
        SuperstructureTuning.periodic();
        break;
      default:
        break;
    }
  }

  /**
   * Call at the end of test mode to end tuning
   */
  public void tuningEnd() {
    switch (tuningMode) {
      case DISABLED:
        break;
      case VOLTAGE:
        GlobalVoltageTuning.end();
        break;
      case SHOOTER:
        ShooterTuning.end();
        break;
      case SUPERSTRUCTURE:
        SuperstructureTuning.end();
        break;
      default:
        break;
    }
  }

  private enum TuningMode {
    DISABLED, VOLTAGE, SHOOTER, SUPERSTRUCTURE
  }
}
