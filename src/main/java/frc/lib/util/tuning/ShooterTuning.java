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

    private static GenericEntry leftSpeed;
    private static GenericEntry rightSpeed;

    public static void initialize(Shooter shooterInstance, Indexer indexerInstance) {

        leftSpeed = Shuffleboard.getTab("ShooterTuning").add("ShooterLeftRPM", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 6000))
                .getEntry();
        rightSpeed = Shuffleboard.getTab("ShooterTuning").add("ShooterRightRPM", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 6000))
                .getEntry();

        shooter = shooterInstance;
        indexer = indexerInstance;
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
            shooter.shooterToRMP(leftSpeed.getDouble(0), rightSpeed.getDouble(0));
        } else {
            shooter.disable();
        }

        Shuffleboard.getTab("ShooterTuning").addDouble("Estimated Target Distance",
                () -> PoseEstimation.getEstimatedPose().getTranslation()
                        .getDistance(FieldUtil.getAllianceSpeakerPosition()))
                .withWidget(BuiltInWidgets.kTextView);
        Shuffleboard.getTab("ShooterTuning").addDouble("LeftVelocityShooter",
                () -> shooter.getLeftVelocity())
                .withWidget(BuiltInWidgets.kTextView);
        Shuffleboard.getTab("ShooterTuning").addDouble("RightVelocityShooter",
                () -> shooter.getRightVelocity())
                .withWidget(BuiltInWidgets.kTextView);

        

    }

    public static void end() {
        shooter.disable();
        indexer.disable();

        leftSpeed.unpublish();
        rightSpeed.unpublish();
    }

}
