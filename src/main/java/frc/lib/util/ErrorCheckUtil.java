package frc.lib.util;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public class ErrorCheckUtil {
    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(StatusCode errorCode, String message) {
        if (errorCode != StatusCode.OK) {
            DriverStation.reportError(message + " : " + errorCode.getName() + " (" + errorCode.getDescription() + ")", false);
        }
    }

    StatusCode lastError = StatusCode.OK;

    /**
     * checks the specified error code and throws an exception if there are any issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkErrorWithThrow(StatusCode errorCode, String message) {
        if (errorCode != StatusCode.OK) {
            throw new RuntimeException(message + " " + errorCode);
        }
    }

    /**
     * Report an error if the given condition is false
     *
     * @param error did it succeed?
     * @param message   message to print if error is false
     */
    public static void checkBoolean(boolean error, String message) {
        if (!error) {
            DriverStation.reportError(message, false);
        }
    }


}
