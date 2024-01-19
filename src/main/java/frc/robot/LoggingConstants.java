// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public final class LoggingConstants {
        public class SwerveLogging {
                public static LoggingLevel Modules = LoggingLevel.BOTH;
                public static LoggingLevel Motors = LoggingLevel.ONBOARD;
                public static LoggingLevel Gyro = LoggingLevel.BOTH;
                public static LoggingLevel Pose = LoggingLevel.NONE;
                public static LoggingLevel Drive = LoggingLevel.NONE;
                public static LoggingLevel PidPose = LoggingLevel.NONE;
                public static LoggingLevel Auto = LoggingLevel.NONE;
                public static LoggingLevel PoseEstimator = LoggingLevel.BOTH;
                public static LoggingLevel Main = LoggingLevel.ONBOARD;
        }

        public class RobotContainerLogging {
                public static LoggingLevel StickValues = LoggingLevel.NONE;
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

        public class ElevatorLogging {
                public static LoggingLevel Main = LoggingLevel.NONE;
                public static LoggingLevel Motor = LoggingLevel.NONE;
                public static LoggingLevel MotionMagic = LoggingLevel.NONE;
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

        public class WristLogging {
                public static LoggingLevel Main = LoggingLevel.NONE;
                public static LoggingLevel Motor = LoggingLevel.NONE;
                public static LoggingLevel MotionMagic = LoggingLevel.NONE;
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

        public class IntakeLogging {
                public static LoggingLevel Main = LoggingLevel.NONE;
                public static LoggingLevel Motor = LoggingLevel.NONE;
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

        public class ShooterLogging {
                public static LoggingLevel Main = LoggingLevel.NONE;
                public static LoggingLevel Motor = LoggingLevel.ONBOARD;
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

        public class GlobalLoggingConstants {
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

}
