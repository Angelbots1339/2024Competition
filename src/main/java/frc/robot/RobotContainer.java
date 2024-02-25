// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.AlignScoreAmp;
import frc.robot.commands.HandOffNote;
import frc.robot.commands.IntakeNoHandoff;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootFromSubwoofer;
import frc.robot.commands.SuperstructureToPosition;
import frc.robot.commands.Auto.AutoShoot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private TuningMode tuningMode = TuningMode.DISABLED;

  // private GenericEntry driveSpeed = Shuffleboard.getTab("Tune").add("DriveSpeed", 0)
  //     .withWidget(BuiltInWidgets.kTextView)
  //     .withProperties(Map.of("min", -1, "max", 1))
  //     .getEntry();

  /***** Instancing Subsystems *****/
  private final Swerve swerve = Constants.GeneratedSwerveConstants.Swerve;
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator();
  private final Indexer indexer = new Indexer();

  private boolean climbMode = false;

  /***** Driver Controls *****/
  private final XboxController driveController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private Supplier<Double> translationX = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftY(), true) * (climbMode ? 0.5 : 1);
  private Supplier<Double> translationY = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftX(), true) * (climbMode ? 0.5 : 1);
  private Supplier<Double> rotation = () -> DriverConstants.fixRotationJoystickValues(-driveController.getRightX(),
      false);

  private Trigger runIntake = new Trigger(() -> driveController.getLeftBumper() && !climbMode);
  private Trigger runIntakeNoHandoff = new Trigger(() -> driveController.getLeftTriggerAxis() > 0.1 && !climbMode);
  private Trigger runOuttake = new Trigger(() -> driveController.getRightBumper() && !climbMode);

  private Trigger subwooferShot = new Trigger(() -> driveController.getAButton() && !climbMode);
  private Trigger subwooferActuallyShoot = new Trigger(() -> driveController.getRightTriggerAxis() > 0.1 && !climbMode);
  private Trigger shootWithRegression = new Trigger(() -> driveController.getXButton() && !climbMode);
  private Trigger scoreAmp = new Trigger(() -> driveController.getBButton() && !climbMode);
  private Trigger alignScoreAmp = new Trigger(() -> driveController.getYButton() && !climbMode);

  private Trigger resetGyro = new Trigger(() -> driveController.getStartButton());
  private Trigger toggleClimbMode = new Trigger(() -> driveController.getBackButton());

  private Trigger extendClimb = new Trigger(() -> driveController.getLeftTriggerAxis() > 0.1 && climbMode);
  private Trigger retractClimb = new Trigger(() -> driveController.getRightTriggerAxis() > 0.1 && climbMode);

  private Trigger extendToMax = new Trigger(() -> driveController.getAButton() && climbMode);

  private void configDriverBindings() {
    resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    toggleClimbMode.onTrue(new InstantCommand(() -> {
      climbMode = !climbMode;
      // Leds.getInstance().climbing = climbMode;
    }));

    runOuttake.whileTrue(new RunCommand(() -> {
      intake.runIntakeDutyCycle(-0.4);
      indexer.runIndexerDutyCycle(-0.4);
    },
        intake, indexer).andThen(new InstantCommand(() -> {
          indexer.disable();
          intake.disable();
        })));

    runIntake.whileTrue(new IntakeNote(intake, indexer, wrist, elevator, () -> false)).onFalse(new HandOffNote(intake, indexer, wrist, elevator));
    runIntakeNoHandoff.whileTrue(new IntakeNoHandoff(intake));

    scoreAmp.whileTrue(new SuperstructureToPosition(elevator, wrist, () -> ScoringConstants.ScoreAmp));
    alignScoreAmp.whileTrue(new AlignScoreAmp(elevator, wrist, indexer, swerve));
    subwooferShot.whileTrue(new ShootFromSubwoofer(elevator, wrist, shooter, swerve, indexer, translationX, translationY, () -> subwooferActuallyShoot.getAsBoolean()));
    shootWithRegression.whileTrue(new Shoot(shooter, wrist, elevator, swerve, indexer, translationX, translationY, () -> false));

    // shoot.whileTrue(swerve.angularDrive(translationX, translationY, () ->
    // Rotation2d.fromDegrees(90), () -> true, () -> true));

    extendClimb.whileTrue(new RunCommand(() -> elevator.setVoltage(driveController.getLeftTriggerAxis() * 10), elevator));
    retractClimb.whileTrue(new RunCommand(() -> elevator.setVoltage(-driveController.getRightTriggerAxis() * 10), elevator));

    extendToMax.onTrue(new RunCommand(() -> elevator.toHeight(ElevatorConstants.maxElevatorHeight - 0.01)));

  }

  private void configOperatorBindings() {

  }

  /***** Initialization *****/
  public RobotContainer() {

    NamedCommands.registerCommand("shoot", new AutoShoot(shooter, wrist,
        elevator, swerve, indexer));
    NamedCommands.registerCommand("intakeNote", new IntakeNote(intake, indexer,
        wrist, elevator, () -> false));
    NamedCommands.registerCommand("runIntakeNoHandoff", new IntakeNoHandoff(intake));

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
      if (climbMode) {
        elevator.holdPosition();
      } else {
        elevator.home();
      }
    }, elevator));
    intake.setDefaultCommand(new InstantCommand(() -> intake.disable(), intake));
    indexer.setDefaultCommand(new InstantCommand(() -> indexer.disable(), indexer));
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
                  // Leds.getInstance().endgameAlert = true;
                  driveController.setRumble(RumbleType.kBothRumble, 1.0);
                  operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                })
                .withTimeout(1.5)
                .andThen(
                    Commands.run(
                        () -> {
                          // Leds.getInstance().endgameAlert = false;
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
                      // Leds.getInstance().endgameAlert = true;
                      driveController.setRumble(RumbleType.kBothRumble, 1.0);
                      operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                    })
                    .withTimeout(0.25),
                Commands.run(
                    () -> {
                      // Leds.getInstance().endgameAlert = false;
                      driveController.setRumble(RumbleType.kBothRumble, 0.0);
                      operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                    })
                    .withTimeout(0.1),
                Commands.run(
                    () -> {
                      // Leds.getInstance().endgameAlert = true;
                      driveController.setRumble(RumbleType.kBothRumble, 1.0);
                      operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                    })
                    .withTimeout(0.25),
                Commands.run(
                    () -> {
                      // Leds.getInstance().endgameAlert = false;
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
