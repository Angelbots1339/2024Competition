// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public class LoggedFalcon extends LoggedObject<TalonFX> {

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, LoggingLevel logType,
            String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, LoggingLevel logType,
            Boolean SeparateTab) {
        super(name, subsystemLogger, object, logType, SeparateTab);
    }

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, LoggingLevel logType) {
        super(name, subsystemLogger, object, logType);
    }

    @Override
    public void initializeShuffleboard() {
        ShuffleboardLayout layout = getTab().getLayout(name, BuiltInLayouts.kList);
        addDoubleToShuffleboard("Stator Current", () ->  object.getStatorCurrent().getValue(), layout);
        addDoubleToShuffleboard("Supply Current", () ->  object.getSupplyCurrent().getValue(), layout);
        addDoubleToShuffleboard("Bus Voltage", () ->  object.getSupplyVoltage().getValue(), layout);
        addDoubleToShuffleboard("Output Voltage", () ->  object.getMotorVoltage().getValue(), layout);
        addDoubleToShuffleboard("Temp", () ->  object.getDeviceTemp().getValue(), layout);

    }

    @Override
    public void initializeDataLog() {
        addDoubleToOnboardLog("Stator Current", () -> object.getStatorCurrent().getValue());
        addDoubleToOnboardLog("Supply Current", () -> object.getSupplyCurrent().getValue());
        addDoubleToOnboardLog("Bus Voltage", () -> object.getSupplyVoltage().getValue());
        addDoubleToOnboardLog("Output Voltage", () -> object.getMotorVoltage().getValue());
        addDoubleToOnboardLog("Temp", () -> object.getDeviceTemp().getValue());
    }



}