// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.util.FieldUtil;
import frc.lib.util.PoseEstimation;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * Class holding all code for tuning the shooter
 */
public class ShooterTuning {

    private static Shooter shooter;
    private static Indexer indexer;

    private static GenericEntry topSpeed;
    private static GenericEntry bottomSpeed;

    public static void initialize(Shooter shooterInstance) {

        topSpeed = Shuffleboard.getTab("Testing").add("Shooter Top Wheel Speed", 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 6000))
                .getEntry();
        bottomSpeed = Shuffleboard.getTab("Testing").add("Shooter Bottom Wheel Speed", 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 6000))
                .getEntry();

        shooter = shooterInstance;
    }

    public static void periodic(XboxController controller) {

        if (controller.getLeftBumper()) {
            indexer.runIndexerDutyCycle(0.3);
        } else if (controller.getRightBumper()) {
            indexer.runIndexerDutyCycle(-0.3);
        } else {
            indexer.disable();
        }

        if (controller.getAButton()) {
            shooter.shooterToRMP(topSpeed.getDouble(0), bottomSpeed.getDouble(0));
        } else {
            shooter.disable();
        }

        Shuffleboard.getTab("Tuning").addDouble("Estimated Target Distance",
                () -> PoseEstimation.getEstimatedPose().getTranslation()
                        .getDistance(FieldUtil.getAllianceSpeakerPosition()))
                .withWidget(BuiltInWidgets.kTextView);

    }

    public static void end() {
        shooter.disable();
        indexer.disable();

        topSpeed.unpublish();
        bottomSpeed.unpublish();
    }

}
