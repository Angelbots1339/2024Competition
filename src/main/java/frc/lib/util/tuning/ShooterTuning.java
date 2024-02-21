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

    private static GenericEntry wristAngle;

    private static XboxController testController;
    private static boolean hasBeenInitialized = false;

    public static void initialize(Shooter shooterInstance, Indexer indexerInstance, Wrist wristInstance) {

        if (!hasBeenInitialized) {
            testController = new XboxController(2);

            leftSpeed = Shuffleboard.getTab("ShooterTuning").add("ShooterLeftRPM", 0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 6000))
                    .getEntry();
            rightSpeed = Shuffleboard.getTab("ShooterTuning").add("ShooterRightRPM", 0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
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
            Shuffleboard.getTab("ShooterTuning").addDouble("LeftRPMShooter",
                    () -> shooter.getLeftVelocity() * 60)
                    .withWidget(BuiltInWidgets.kTextView);
            Shuffleboard.getTab("ShooterTuning").addDouble("RightRPMShooter",
                    () -> shooter.getRightVelocity() * 60)
                    .withWidget(BuiltInWidgets.kTextView);

            shooter = shooterInstance;
            indexer = indexerInstance;
            wrist = wristInstance;

            hasBeenInitialized = true;
        }

    }

    public static void periodic() {

        if (testController.getLeftBumper()) {
            indexer.runIndexerDutyCycle(0.3);
        } else if (testController.getRightBumper()) {
            indexer.runIndexerDutyCycle(-0.3);
        } else {
            indexer.disable();
        }

        if (testController.getAButton()) {
            // shooter.shooterToRMP(leftSpeed.getDouble(6000), rightSpeed.getDouble(4500));
            shooter.shooterToRMP(6000, 5000);
            wrist.toAngle(Rotation2d.fromDegrees(wristAngle.getDouble(90)));

        } else {
            shooter.disable();
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
