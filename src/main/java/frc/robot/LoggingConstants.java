// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public final class LoggingConstants {
        // Shuffleboard automatically logs to both Onboard and Network Tables
        // Make sure you call everything in Robot.java that needs to be called
        public class SwerveLogging {
                public static LoggingLevel Modules = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Motors = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Gyro = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Pose = LoggingLevel.NETWORK_TABLES;
                public static LoggingLevel PidPose = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Auto = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Main = LoggingLevel.ONBOARD_ONLY;
        }

        public class RobotContainerLogging {
                public static LoggingLevel StickValues = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Shooting = LoggingLevel.NETWORK_TABLES;
        }

        public class ElevatorLogging {
                public static LoggingLevel Main = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Motor = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel MotionMagic = LoggingLevel.NONE;
        }

        public class WristLogging {
                public static LoggingLevel Main = LoggingLevel.NETWORK_TABLES;
                public static LoggingLevel Motor = LoggingLevel.ONBOARD_ONLY;
        }

        public class IntakeLogging {
                public static LoggingLevel Main = LoggingLevel.NETWORK_TABLES;
                public static LoggingLevel Motor = LoggingLevel.ONBOARD_ONLY;
        }

        public class IndexerLogging {
                public static LoggingLevel Main = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Motor = LoggingLevel.ONBOARD_ONLY;
        }

        public class ShooterLogging {
                public static LoggingLevel Main = LoggingLevel.ONBOARD_ONLY;
                public static LoggingLevel Motor = LoggingLevel.ONBOARD_ONLY;
        }

        public class GlobalLoggingConstants {
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

}
