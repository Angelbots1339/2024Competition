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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team254.math.InterpolatingDouble;
import frc.lib.util.FieldUtil;
import frc.lib.util.Leds;
import frc.lib.util.PoseEstimation;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.tuning.GlobalVoltageTuning;
import frc.lib.util.tuning.ShooterTuning;
import frc.lib.util.tuning.SuperstructureTuning;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.HandOffNote;
import frc.robot.commands.IntakeNoHandoff;
import frc.robot.commands.IntakeNote;
import frc.robot.LoggingConstants.RobotContainerLogging;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootFromSubwoofer;
import frc.robot.commands.Auto.AutoShoot;
import frc.robot.commands.Auto.AutoSpinUp;
import frc.robot.regressions.SpeakerShotRegression;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private final LoggedSubsystem logger = new LoggedSubsystem("RobotContainer");
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private TuningMode tuningMode = TuningMode.SUPERSTRUCTURE;

  // private GenericEntry wristAngle =
  // Shuffleboard.getTab("Tune").add("WristAngle", 90)
  // .withWidget(BuiltInWidgets.kTextView)
  // .withProperties(Map.of("min", -25, "max", 90))
  // .getEntry();

  /***** Instancing Subsystems *****/
  private final Swerve swerve = Constants.SwerveConstants.Swerve;
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator();
  private final Indexer indexer = new Indexer();

  public boolean climbMode = false;

  /***** Driver Controls *****/
  private final XboxController driveController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private Supplier<Double> translationX = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftY(), true) * (climbMode ? 0.5 : 1);
  private Supplier<Double> translationY = () -> DriverConstants
      .fixTranslationJoystickValues(-driveController.getLeftX(), true) * (climbMode ? 0.5 : 1);
  private Supplier<Double> rotation = () -> DriverConstants.fixRotationJoystickValues(-driveController.getRightX(),
      false);

  private Supplier<Double> manualWristRotation = () -> driveController.getRightY();

  private Trigger runIntake = new Trigger(() -> driveController.getLeftBumper() && !climbMode);
  private Trigger runOuttake = new Trigger(
    () -> driveController.getRightBumper() && !climbMode && !driveController.getAButton());
    
    private Trigger subwooferShot = new Trigger(() -> driveController.getAButton() && !climbMode);
    private Trigger actuallyShoot = new Trigger(() -> driveController.getRightTriggerAxis() > 0.1 && !climbMode);
    private Trigger shootWithRegression = new Trigger(() -> driveController.getXButton() && !climbMode);
    private Trigger scoreAmp = new Trigger(() -> driveController.getBButton() && !climbMode);

    private Trigger sourceShuttle = new Trigger(() -> driveController.getYButton() && !climbMode);
    private Trigger lowShuttle = new Trigger(() -> driveController.getLeftTriggerAxis() > 0.1 && !climbMode);

  private Trigger resetGyro = new Trigger(() -> driveController.getStartButton());
  private Trigger toggleClimbMode = new Trigger(() -> operatorController.getBackButton());

  private Trigger extendClimb = new Trigger(() -> driveController.getLeftTriggerAxis() > 0.1 && climbMode);
  private Trigger retractClimb = new Trigger(() -> driveController.getRightTriggerAxis() > 0.1 && climbMode);

  private Trigger extendToMax = new Trigger(() -> driveController.getAButton() && climbMode);

  private Trigger zeroWrist = new Trigger(() -> operatorController.getAButton());

  private void configDriverBindings() {
    resetGyro.onTrue(new InstantCommand(() -> swerve.zeroGyroAdjusted()));

    runOuttake.whileTrue(new RunCommand(() -> {
      intake.setVoltage(ScoringConstants.outtakingTargetVoltage);

      if (scoreAmp.getAsBoolean()) {
        indexer.setVoltage(-ScoringConstants.indexerScoringVoltage);
      }
    },
        intake, indexer).andThen(new InstantCommand(() -> {
          indexer.disable();
          intake.disable();
        })));

    runIntake.whileTrue(new IntakeNote(intake, indexer, wrist, elevator))
        .onFalse(new HandOffNote(intake, indexer, wrist, elevator));

    scoreAmp.whileTrue(
        new ScoreAmp(elevator, wrist, indexer, swerve, translationX, translationY));

    subwooferShot.whileTrue(new ShootFromSubwoofer(elevator, wrist, shooter, swerve, indexer, translationX,
        translationY, actuallyShoot::getAsBoolean));
    shootWithRegression.whileTrue(
        new Shoot(shooter, wrist, elevator, swerve, indexer, translationX, translationY, actuallyShoot::getAsBoolean));

    sourceShuttle.whileTrue(new RunCommand(() -> {

      wrist.toAngle(ScoringConstants.SourceShuttleShot.angle);
      elevator.toHeight(ScoringConstants.SourceShuttleShot.height);
      shooter.shooterToRMP(3500, 4500);

      swerve.angularDriveRequest(() -> translationX.get(),
          () -> translationY.get(),
          () -> Rotation2d.fromDegrees(FieldUtil.isAllianceBlue() ? -33 : 33), () -> true);

      if (actuallyShoot.getAsBoolean()) {
        indexer.setVoltage(ScoringConstants.indexingTargetVolts);
      } else {
        indexer.disable();
      }

    }, shooter, wrist, elevator, swerve, indexer));

    lowShuttle.whileTrue(new RunCommand(() -> {

      wrist.toAngle(ScoringConstants.LowShuttleShot.angle);
      elevator.toHeight(ScoringConstants.SourceShuttleShot.height);
      shooter.shooterToRMP(5000, 6000);

      swerve.angularDriveRequest(() -> translationX.get(),
          () -> translationY.get(),
          () -> Rotation2d.fromDegrees(0), () -> true);

      if (actuallyShoot.getAsBoolean()) {
        indexer.setVoltage(ScoringConstants.indexingTargetVolts);
      } else {
        indexer.disable();
      }

    }, shooter, wrist, elevator, swerve, indexer));
    
  }

  private void configOperatorBindings() {
    toggleClimbMode.onTrue(new InstantCommand(() -> {
      climbMode = !climbMode;

      if(climbMode) {
        wrist.toAngle(ScoringConstants.WristClimbPos.angle);
      } else {
        wrist.home();
      }
      Leds.getInstance().climbing = climbMode;
    }, elevator));

    zeroWrist.onTrue(new InstantCommand(() -> wrist.resetToAbsolute()));
  }

  /***** Initialization *****/
  public RobotContainer() {

    Leds.getInstance();
    initializeLogging();

    NamedCommands.registerCommand("shoot", new AutoShoot(shooter, wrist,
        elevator, swerve, indexer, true, true));
    NamedCommands.registerCommand("shootNoAlign", new AutoShoot(shooter, wrist,
        elevator, swerve, indexer, true, false));
    NamedCommands.registerCommand("shootNoVision", new AutoShoot(shooter, wrist,
        elevator, swerve, indexer, false, false));
    NamedCommands.registerCommand("spinUp", new AutoSpinUp(shooter, wrist,
        elevator, swerve, indexer, true));
    NamedCommands.registerCommand("spinUpNoVision", new AutoSpinUp(shooter, wrist,
        elevator, swerve, indexer, false));
    NamedCommands.registerCommand("intakeNote", new IntakeNote(intake, indexer,
        wrist, elevator).andThen(new HandOffNote(intake, indexer, wrist, elevator)));
    NamedCommands.registerCommand("runIntakeNoHandoff", new IntakeNoHandoff(intake));
    NamedCommands.registerCommand("finishIntaking", Commands.either(Commands.none(),
        new IntakeNote(intake, indexer,
            wrist, elevator).andThen(new HandOffNote(intake, indexer, wrist, elevator)),
        () -> indexer.isNoteAtTarget()));
    NamedCommands.registerCommand("vision", new RunCommand(() -> swerve.updateVision()));
    NamedCommands.registerCommand("runShooter", new RunCommand(
        () -> shooter.shooterToRMP(ScoringConstants.shooterSetpointFar[0], ScoringConstants.shooterSetpointFar[1]),
        shooter));
    NamedCommands.registerCommand("bloop", new RunCommand(() -> {
      shooter.shooterToRMP(1000);
      indexer.setVoltage(ScoringConstants.indexingTargetVolts);
    }).until(() -> !indexer.isNotePresent()));

    // autoChooser.addOption("Mobility", new PathPlannerAuto("Mobility"));
    // autoChooser.addOption("Shoot1Center", new PathPlannerAuto("Shoot1Center"));
    // autoChooser.addOption("Center2Piece", new PathPlannerAuto("Center2Piece"));
    // autoChooser.addOption("Center3PieceClose", new
    // PathPlannerAuto("Center3PieceClose"));
    // autoChooser.addOption("AmpTop2Piece", new PathPlannerAuto("AmpTop2Piece"));

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configDefaultCommands();
    configDriverBindings();
    configOperatorBindings();
    initializeEndgameAlerts();
  }

  private void configDefaultCommands() {

    swerve.setDefaultCommand(swerve.drive(translationX, translationY, rotation, () -> true, () -> true));
    wrist.setDefaultCommand(Commands.either(new InstantCommand(() -> wrist.toAngle(ScoringConstants.WristClimbPos.angle), wrist),
        new InstantCommand(wrist::home, wrist), () -> climbMode));
    elevator
        .setDefaultCommand(Commands.either(new ManualClimb(elevator, wrist, () -> driveController.getLeftTriggerAxis(),
            () -> driveController.getRightTriggerAxis(),
            manualWristRotation), new InstantCommand(elevator::home, elevator), () -> climbMode));
    intake.setDefaultCommand(new InstantCommand(intake::disable, intake));
    indexer.setDefaultCommand(new InstantCommand(indexer::disable, indexer));
    shooter.setDefaultCommand(new InstantCommand(shooter::disable, shooter));
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

  /**
   * Should be called periodically from RobotPeriodic
   */
  public void updateDashboard() {
    SmartDashboard.putBoolean("Climb Mode", climbMode);
  }

  public void initializeLogging() {
    Supplier<Double> distance = () -> PoseEstimation.getEstimatedPose().getTranslation()
        .getDistance(FieldUtil.getAllianceSpeakerPosition());

    logger.addDouble("PolyRegressionAngle", () -> SpeakerShotRegression.wristRegression.predict(distance.get()),
        RobotContainerLogging.Shooting);
    logger.addDouble("LinearRegressionAngle", () -> SpeakerShotRegression.wristExpoRegression(distance.get()),
        RobotContainerLogging.Shooting);
    logger.addDouble("InterpolationAngle",
        () -> SpeakerShotRegression.wristInterpolation.getInterpolated(new InterpolatingDouble(distance.get())).value,
        RobotContainerLogging.Shooting);

    logger.addDouble("SpeakerDistance", () -> distance.get(), RobotContainerLogging.Shooting);

    Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians( // Find the angle to turn the robot to
        Math.atan((PoseEstimation.getEstimatedPose().getY() - FieldUtil.getAllianceSpeakerPosition().getY())
            / (PoseEstimation.getEstimatedPose().getX() - FieldUtil.getAllianceSpeakerPosition().getX())));

    logger.addDouble("TargetRobotAngle", () -> robotAngle.get().getDegrees(),
        RobotContainerLogging.Shooting);

    logger.addBoolean("ShooterAtSetpoint", () -> shooter.isAtSetpoint(), RobotContainerLogging.Shooting);
    logger.addBoolean("WristAtSetpoint", () -> wrist.isAtSetpoint(), RobotContainerLogging.Shooting);
    logger.addBoolean("ElevatorAtSetpoint", () -> elevator.isAtSetpoint(), RobotContainerLogging.Shooting);
    logger.addBoolean("AngularDriveAtSetpoint", () -> swerve.isAngularDriveAtSetpoint(),
        RobotContainerLogging.Shooting);

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
