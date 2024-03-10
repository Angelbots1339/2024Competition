// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.util.FieldUtil;
import frc.lib.util.PoseEstimation;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

/**
 * Class holding all code for tuning the shooter
 */
public class ShooterTuning {

    private static Shooter shooter;
    private static Indexer indexer;
    private static Wrist wrist;

    private static GenericEntry leftSpeed;
    private static GenericEntry rightSpeed;

    private static double leftRPM;
    private static double rightRPM;
    private static double wristTargetAngle;

    private static GenericEntry wristAngle;

    private static XboxController testController;
    private static boolean hasBeenInitialized = false;

    public static void initialize(Shooter shooterInstance, Indexer indexerInstance, Wrist wristInstance) {

        if (!hasBeenInitialized) {
            testController = new XboxController(2);

            leftSpeed = Shuffleboard.getTab("ShooterTuning").add("LeftTargetRPM", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withProperties(Map.of("min", 0, "max", 6000))
                    .getEntry();
            rightSpeed = Shuffleboard.getTab("ShooterTuning").add("RightTargetRPM", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withProperties(Map.of("min", 0, "max", 6000))
                    .getEntry();
            wristAngle = Shuffleboard.getTab("ShooterTuning").add("WristAngle", 90)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withProperties(Map.of("min", -25, "max", 180))
                    .getEntry();

            Shuffleboard.getTab("ShooterTuning").addDouble("Estimated Target Distance",
                    () -> PoseEstimation.getEstimatedPose().getTranslation()
                            .getDistance(FieldUtil.getAllianceSpeakerPosition()))
                    .withWidget(BuiltInWidgets.kTextView);
            Shuffleboard.getTab("ShooterTuning").addDouble("LeftActualRPM",
                    () -> shooter.getLeftVelocity() * 60)
                    .withWidget(BuiltInWidgets.kTextView);
            Shuffleboard.getTab("ShooterTuning").addDouble("RightActualRPM",
                    () -> shooter.getRightVelocity() * 60)
                    .withWidget(BuiltInWidgets.kTextView);
            Shuffleboard.getTab("ShooterTuning").addDouble("WristActualAngle",
                    () -> wrist.getAngle().getDegrees())
                    .withWidget(BuiltInWidgets.kTextView);

            shooter = shooterInstance;
            indexer = indexerInstance;
            wrist = wristInstance;

            hasBeenInitialized = true;
        }

    }

    public static void periodic() {

        if (leftSpeed.isValid()) {
            leftRPM = leftSpeed.getDouble(0);
        }
        if (rightSpeed.isValid()) {
            rightRPM = rightSpeed.getDouble(0);
        }
        if (wristAngle.isValid()) {
            wristTargetAngle = wristAngle.getDouble(0);
        }

        // shooter.shooterToRMP(leftRPM, rightRPM);
        // wrist.toAngle(Rotation2d.fromDegrees(wristTargetAngle));

        if (testController.getLeftBumper() && !testController.getAButton()) {
            // indexer.runIndexerDutyCycle(0.2);
            indexer.indexNoteToTarget();
        } else if (testController.getLeftBumper() && testController.getAButton()) {
            indexer.runIndexerDutyCycle(0.3);
        } else if (testController.getRightBumper()) {
            indexer.runIndexerDutyCycle(-0.2);
        } else {
            indexer.disable();
        }

        if (testController.getBButton()) {
            wrist.toAngle(Rotation2d.fromDegrees(150));
        } else if (testController.getAButton()) {
            shooter.shooterToRMP(leftRPM, rightRPM);
            wrist.toAngle(Rotation2d.fromDegrees(wristTargetAngle));

        } else {
            shooter.disable();
            wrist.home();
        }

        // indexer.runIndexerDutyCycle(0.3);
        // shooter.shooterToRMP(leftSpeed.getDouble(0), rightSpeed.getDouble(0));

    }

    public static void end() {
        shooter.disable();
        indexer.disable();

        // leftSpeed.unpublish();
        // rightSpeed.unpublish();
    }

}
