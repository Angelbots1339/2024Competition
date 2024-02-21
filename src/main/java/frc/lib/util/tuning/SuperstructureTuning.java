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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Wrist;

/**
 * Class holding all code for tuning the shooter
 */
public class SuperstructureTuning {

        private static Elevator elevator;
        private static Wrist wrist;
        private static Indexer indexer;

        private static GenericEntry wristTargetAngle;
        private static GenericEntry elevatorTargetHeight;
        private static GenericEntry indexerSpeed;

        private static XboxController testController;
        private static boolean hasBeenInitialized = false;

        public static void initialize(Elevator elevatorInstance, Wrist wristInstance, Indexer indexerInstance) {

                if (!hasBeenInitialized) {
                        testController = new XboxController(2);

                        wristTargetAngle = Shuffleboard.getTab("SuperstructureTuning").add("WristTargetAngle", 90)
                                        .withWidget(BuiltInWidgets.kNumberSlider)
                                        .withProperties(Map.of("min", WristConstants.wristMinAngle.getDegrees(), "max",
                                                        WristConstants.wristMaxAngle.getDegrees()))
                                        .getEntry();
                        elevatorTargetHeight = Shuffleboard.getTab("SuperstructureTuning")
                                        .add("ElevatorTargetHeight", 0)
                                        .withWidget(BuiltInWidgets.kNumberSlider)
                                        .withProperties(Map.of("min", 0, "max", ElevatorConstants.maxElevatorHeight))
                                        .getEntry();
                        indexerSpeed = Shuffleboard.getTab("SuperstructureTuning")
                                        .add("IndexerVolts", 0)
                                        .withWidget(BuiltInWidgets.kNumberSlider)
                                        .withProperties(Map.of("min", -1, "max", 1))
                                        .getEntry();

                        Shuffleboard.getTab("SuperstructureTuning").addDouble("WristSetpointError",
                                        () -> wrist.getSetpointError().getDegrees())
                                        .withWidget(BuiltInWidgets.kTextView);
                        Shuffleboard.getTab("SuperstructureTuning").addDouble("WristAngle",
                                        () -> wrist.getAngle().getDegrees())
                                        .withWidget(BuiltInWidgets.kTextView);
                        Shuffleboard.getTab("SuperstructureTuning").addDouble("WristEncoderAbsolutePosition",
                                        () -> wrist.getAbsoluteEncoderPosition().getDegrees())
                                        .withWidget(BuiltInWidgets.kTextView);
                        Shuffleboard.getTab("SuperstructureTuning").addDouble("WristVoltsOUt",
                                        () -> wrist.getVoltageOut())
                                        .withWidget(BuiltInWidgets.kTextView);

                        Shuffleboard.getTab("SuperstructureTuning").addDouble("ElevatorSetpointError",
                                        () -> elevator.getSetpointError())
                                        .withWidget(BuiltInWidgets.kTextView);
                        Shuffleboard.getTab("SuperstructureTuning").addDouble("ElevatorPosition",
                                        () -> elevator.getPosition())
                                        .withWidget(BuiltInWidgets.kTextView);

                        elevator = elevatorInstance;
                        wrist = wristInstance;
                        indexer = indexerInstance;

                        hasBeenInitialized = true;

                }
        }

        public static void periodic() {

                if (testController.getAButton()) {
                        // elevator.toHeight(elevatorTargetHeight.getDouble(0));
                        // wrist.toAngle(wristTargetAngle.getDouble(0));
                } else {
                        // elevator.disable();
                        // wrist.disable();
                }
                
                indexer.runIndexerDutyCycle(indexerSpeed.getDouble(0));
                elevator.toHeight(elevatorTargetHeight.getDouble(0));
                wrist.toAngle(Rotation2d.fromDegrees(wristTargetAngle.getDouble(0)));
                System.out.println(wristTargetAngle.getDouble(0));
        }

        public static void end() {
                elevator.disable();
                wrist.disable();
                indexer.disable();
        }

}
