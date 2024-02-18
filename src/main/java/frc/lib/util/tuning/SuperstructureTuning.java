// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/**
 * Class holding all code for tuning the shooter
 */
public class SuperstructureTuning {

        private static Elevator elevator;
        private static Wrist wrist;

        private static GenericEntry wristTargetAngle;
        private static GenericEntry elevatorTargetHeight;

        private static XboxController testController;
        private static boolean hasBeenInitialized = false;

        public static void initialize(Elevator elevatorInstance, Wrist wristInstance) {

                if (!hasBeenInitialized) {
                        testController = new XboxController(2);

                        wristTargetAngle = Shuffleboard.getTab("SuperstructureTuning").add("WristTargetAngle", 90)
                                        .withWidget(BuiltInWidgets.kNumberSlider)
                                        .withProperties(Map.of("min", 0, "max", 180))
                                        .getEntry();
                        elevatorTargetHeight = Shuffleboard.getTab("SuperstructureTuning")
                                        .add("ElevatorTargetHeight", 0)
                                        .withWidget(BuiltInWidgets.kNumberSlider)
                                        .withProperties(Map.of("min", 0, "max", 1))
                                        .getEntry();

                        elevator = elevatorInstance;
                        wrist = wristInstance;

                        hasBeenInitialized = true;

                }
        }

        public static void periodic() {

                if (testController.getAButton()) {
                        elevator.toHeight(elevatorTargetHeight.getDouble(0));
                        wrist.toAngle(wristTargetAngle.getDouble(0));
                } else {
                        elevator.disable();
                        wrist.disable();
                }

                Shuffleboard.getTab("SuperstructureTuning").addDouble("WristSetpointError",
                                () -> wrist.getSetpointError())
                                .withWidget(BuiltInWidgets.kTextView);
                Shuffleboard.getTab("SuperstructureTuning").addDouble("WristAngle",
                                () -> wrist.getAngle().getDegrees())
                                .withWidget(BuiltInWidgets.kTextView);
                Shuffleboard.getTab("SuperstructureTuning").addDouble("WristEncoderAbsolutePosition",
                                () -> wrist.getAbsoluteEncoderPosition().getDegrees())
                                .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("SuperstructureTuning").addDouble("ElevatorSetpointError",
                                () -> elevator.getSetpointError())
                                .withWidget(BuiltInWidgets.kTextView);
                Shuffleboard.getTab("SuperstructureTuning").addDouble("ElevatorPosition",
                                () -> elevator.getPosition())
                                .withWidget(BuiltInWidgets.kTextView);

        }

        public static void end() {
                elevator.disable();
                wrist.disable();
        }

}
