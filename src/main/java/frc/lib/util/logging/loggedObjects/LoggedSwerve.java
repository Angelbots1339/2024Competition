// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.loggedPrimitives.LoggedDoubleArray;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

/** Add your docs here. */
public class LoggedSwerve extends LoggedObject<Swerve> {

    private SwerveDriveState currentState = new SwerveDriveState();

    public void setCurrentState(SwerveDriveState newState) {
        currentState = newState;
    }

    public LoggedSwerve(String name, LoggedContainer subsystemLogger, Swerve object, String logType,
            String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
        object.registerTelemetry(this::setCurrentState);
    }

    public LoggedSwerve(String name, LoggedContainer subsystemLogger, Swerve object, String logType,
            Boolean SeparateTab) {
        super(name, subsystemLogger, object, logType, SeparateTab);
        object.registerTelemetry(this::setCurrentState);
    }

    public LoggedSwerve(String name, LoggedContainer subsystemLogger, Swerve object, String logType) {
        super(name, subsystemLogger, object, logType);
        object.registerTelemetry(this::setCurrentState);
    }

    
    public double driveRotsToMeters(double rots) {
        return rots * 2 * Math.PI * SwerveConstants.kWheelRadiusInches;
    }

    @Override
    protected void initializeShuffleboard() {


        ShuffleboardLayout layout0 = getTab().getLayout("Module:0", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.getModule(0).getCANcoder().getPosition().getValue(), layout0);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[0].angle.getRotations(), layout0);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[0].speedMetersPerSecond, layout0);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.getModule(0).getDriveMotor().getPosition().getValue()), layout0);
        addDoubleToShuffleboard("TotalRotations", () -> object.getModule(0).getDriveMotor().getPosition().getValue(), layout0);

        ShuffleboardLayout layout1 = getTab().getLayout("Module:1", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.getModule(1).getCANcoder().getPosition().getValue(), layout1);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[1].angle.getRotations(), layout1);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[1].speedMetersPerSecond, layout1);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.getModule(1).getDriveMotor().getPosition().getValue()), layout1);
        addDoubleToShuffleboard("TotalRotations", () -> object.getModule(1).getDriveMotor().getPosition().getValue(), layout1);

        ShuffleboardLayout layout2 = getTab().getLayout("Module:2", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.getModule(2).getCANcoder().getPosition().getValue(), layout2);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[2].angle.getRotations(), layout2);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[2].speedMetersPerSecond, layout2);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.getModule(2).getDriveMotor().getPosition().getValue()), layout2);
        addDoubleToShuffleboard("TotalRotations", () -> object.getModule(2).getDriveMotor().getPosition().getValue(), layout2);

        ShuffleboardLayout layout3 = getTab().getLayout("Module:3", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.getModule(3).getCANcoder().getPosition().getValue(), layout3);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[3].angle.getRotations(), layout3);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[3].speedMetersPerSecond, layout3);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.getModule(3).getDriveMotor().getPosition().getValue()), layout3);
        addDoubleToShuffleboard("TotalRotations", () -> object.getModule(3).getDriveMotor().getPosition().getValue(), layout3);

        // FL, FR, BL, BR
        ShuffleboardLayout trueStatesLayout = getTab().getLayout("TrueModuleStates", BuiltInLayouts.kList).withSize(2,
                3);
        addDoubleArrayToShuffleboard(name, () -> new double[] {
                currentState.ModuleStates[0].angle.getDegrees(), currentState.ModuleStates[0].speedMetersPerSecond,
                currentState.ModuleStates[1].angle.getDegrees(), currentState.ModuleStates[1].speedMetersPerSecond,
                currentState.ModuleStates[2].angle.getDegrees(), currentState.ModuleStates[2].speedMetersPerSecond,
                currentState.ModuleStates[3].angle.getDegrees(), currentState.ModuleStates[3].speedMetersPerSecond

        }, trueStatesLayout);

        // System.out.println(object[0].getState());

        ShuffleboardLayout desiredStatesLayout = getTab().getLayout("DesiredModuleStates", BuiltInLayouts.kList)
                .withSize(2, 3);
        addDoubleArrayToShuffleboard(name, () -> new double[] {
                currentState.ModuleTargets[0].angle.getDegrees(), currentState.ModuleTargets[0].speedMetersPerSecond,
                currentState.ModuleTargets[1].angle.getDegrees(), currentState.ModuleTargets[1].speedMetersPerSecond,
                currentState.ModuleTargets[2].angle.getDegrees(), currentState.ModuleTargets[2].speedMetersPerSecond,
                currentState.ModuleTargets[3].angle.getDegrees(), currentState.ModuleTargets[3].speedMetersPerSecond
        }, desiredStatesLayout);

        // FL, FR, BL, BR
        add(new LoggedDoubleArray(this, () -> new double[] {
                currentState.ModuleStates[0].angle.getDegrees(), currentState.ModuleStates[0].speedMetersPerSecond,
                currentState.ModuleStates[1].angle.getDegrees(), currentState.ModuleStates[1].speedMetersPerSecond,
                currentState.ModuleStates[2].angle.getDegrees(), currentState.ModuleStates[2].speedMetersPerSecond,
                currentState.ModuleStates[3].angle.getDegrees(), currentState.ModuleStates[3].speedMetersPerSecond

        }, NetworkTableInstance.getDefault().getTable(getPrefix()).getSubTable(name).getDoubleArrayTopic("TrueStates")
                .getGenericEntry()));

        add(new LoggedDoubleArray(this, () -> new double[] {
                currentState.ModuleTargets[0].angle.getDegrees(), currentState.ModuleTargets[0].speedMetersPerSecond,
                currentState.ModuleTargets[1].angle.getDegrees(), currentState.ModuleTargets[1].speedMetersPerSecond,
                currentState.ModuleTargets[2].angle.getDegrees(), currentState.ModuleTargets[2].speedMetersPerSecond,
                currentState.ModuleTargets[3].angle.getDegrees(), currentState.ModuleTargets[3].speedMetersPerSecond

        }, NetworkTableInstance.getDefault().getTable(getPrefix()).getSubTable(name)
                .getDoubleArrayTopic("DesiredStates").getGenericEntry()));

    }

    @Override
    protected void initializeDataLog() {


        

        addDoubleToOnboardLog("Module:0/CanCoder", () -> object.getModule(0).getCANcoder().getPosition().getValue());
        addDoubleToOnboardLog("Module:0/TotalDistance", () -> driveRotsToMeters(object.getModule(0).getDriveMotor().getPosition().getValue()));

        addDoubleToOnboardLog("Module:1/CanCoder", () -> object.getModule(1).getCANcoder().getPosition().getValue());
        addDoubleToOnboardLog("Module:1/TotalDistance", () -> driveRotsToMeters(object.getModule(1).getDriveMotor().getPosition().getValue()));

        addDoubleToOnboardLog("Module:2/CanCoder", () -> object.getModule(2).getCANcoder().getPosition().getValue());
        addDoubleToOnboardLog("Module:2/TotalDistance", () -> driveRotsToMeters(object.getModule(2).getDriveMotor().getPosition().getValue()));

        addDoubleToOnboardLog("Module:3/CanCoder", () -> object.getModule(3).getCANcoder().getPosition().getValue());
        addDoubleToOnboardLog("Module:3/TotalDistance", () -> driveRotsToMeters(object.getModule(3).getDriveMotor().getPosition().getValue()));

        addDoubleArrayToOnboardLog("TrueStates", () -> new double[] {
                currentState.ModuleStates[0].angle.getDegrees(), currentState.ModuleStates[0].speedMetersPerSecond,
                currentState.ModuleStates[1].angle.getDegrees(), currentState.ModuleStates[1].speedMetersPerSecond,
                currentState.ModuleStates[2].angle.getDegrees(), currentState.ModuleStates[2].speedMetersPerSecond,
                currentState.ModuleStates[3].angle.getDegrees(), currentState.ModuleStates[3].speedMetersPerSecond
        });

        addDoubleArrayToOnboardLog("DesiredStates", () -> new double[] {
                currentState.ModuleTargets[0].angle.getDegrees(), currentState.ModuleTargets[0].speedMetersPerSecond,
                currentState.ModuleTargets[1].angle.getDegrees(), currentState.ModuleTargets[1].speedMetersPerSecond,
                currentState.ModuleTargets[2].angle.getDegrees(), currentState.ModuleTargets[2].speedMetersPerSecond,
                currentState.ModuleTargets[3].angle.getDegrees(), currentState.ModuleTargets[3].speedMetersPerSecond
        });
    }

}
