// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class LoggedCommand extends LoggedContainer{
    public LoggedCommand(String name) {
        super(name);
    }

    public void initialize(){
        DataLogManager.log("Command|" + name + "|Start|" + Timer.getFPGATimestamp());
    }
    public void stopLogging(){
        DataLogManager.log("Command|" + name + "|End|" + Timer.getFPGATimestamp());;
        Logger.getInstance().removeContainer(this);
    }


}
