// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedPrimitives.LoggedBoolean;
import frc.lib.util.logging.loggedPrimitives.LoggedDouble;
import frc.lib.util.logging.loggedPrimitives.LoggedInteger;
import frc.lib.util.logging.loggedPrimitives.LoggedPrimitive;
import frc.lib.util.logging.loggedPrimitives.LoggedString;


/** Add your docs here. */
public class LoggedContainer implements Iloggable {
    private Map<String, LoggedPrimitive<?>> loggedPrimitives = new HashMap<String, LoggedPrimitive<?>>();

    protected final String name;

    public LoggedContainer(String name) {
        this.name = name;
        Logger.getInstance().addContainer(this);

    }

    public void log(long timestamp) {
        loggedPrimitives.forEach((name, loggedPrimitive) -> loggedPrimitive.log(timestamp));
    }

    

    public void addString(String name, Supplier<String> supplier, LoggingLevel logType) {
        loggedPrimitives.put(name, new LoggedString(name, logType, this, supplier));
    }
    public void addDouble(String name, Supplier<Double> supplier, LoggingLevel logType) {
        loggedPrimitives.put(name, new LoggedDouble(name, logType, this, supplier));
    }
    public void addBoolean(String name, Supplier<Boolean> supplier, LoggingLevel logType) {
        loggedPrimitives.put(name, new LoggedBoolean(name, logType, this, supplier));
    }
    public void addInteger(String name, Supplier<Integer> supplier, LoggingLevel logType) {
        loggedPrimitives.put(name, new LoggedInteger(name, logType, this, supplier));
    }

    public void updateDouble(String name, double value, LoggingLevel logType) {
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedDouble(name, logType, this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedDouble){
            ((LoggedDouble) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
    }
    public void updateBoolean(String name, boolean value, LoggingLevel logType) {
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedBoolean(name, logType, this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedBoolean){
            ((LoggedBoolean) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
        
    }
    public void updateString(String name, String value, LoggingLevel logType) {
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedString(name, logType, this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedString){
            ((LoggedString) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
    }
    public void updateInteger(String name, int value, LoggingLevel logType) {
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedInteger(name, logType, this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedInteger){
            ((LoggedInteger) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
    }

    public boolean isLogging(LoggingLevel logType){
        return logType != LoggingLevel.NONE;
    }

    public String getName() {
        return name;
    }

}
