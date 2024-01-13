// Modified from 254's TalonFXFactory to use the Phoenix 6 API
// See https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/TalonFXFactory.java

package frc.lib.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix.ParamEnum;
// import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.lib.util.ErrorCheckUtil.CommonErrorNames;


/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    private final static double kTimeoutSeconds = 0.1;
    public static int kTimeoutMs = 0;


    // public static class Configuration {
    //     public NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    //     // factory defaults
    //     public double NEUTRAL_DEADBAND = 0.04;

    //     public boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
    //     public boolean ENABLE_STATOR_CURRENT_LIMIT = true;
    //     public int SUPPLY_CURRENT_LIMIT = 40;
    //     public int STATOR_CURRENT_LIMIT = 80;
    

    //     public double OPEN_LOOP_RAMP_RATE = 0.0;
    //     public double CLOSED_LOOP_RAMP_RATE = 0.0;
    // }
    

    private static final TalonFXConfiguration kDefaultConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast));


    private static final TalonFXConfiguration kFollowerConfiguration = kDefaultConfiguration;

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically. Potentially try to increase as much as possible.
       
    }

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(int id, String canBus) {
        return createTalon(id, canBus, kDefaultConfiguration);
    }

    // public static WPI_TalonFX createDefaultSimulationTalon(int id, String canBus) {
    //     return createSimulationTalon(id, canBus, kDefaultConfiguration);
    // }

    /**
     * 
     * @param follower_id
     * @param leader_id
     * @param canBus
     * @param invertedFromMaster Set to true for the motor to spin the opposite way of the master
     * @return TalonFX
     */
    public static TalonFX createPermanentFollowerTalon(int follower_id, int leader_id, String canBus, Boolean invertedFromMaster) {
        final TalonFX talon = createTalon(follower_id, canBus, kFollowerConfiguration);
        talon.setControl(new Follower(leader_id, invertedFromMaster));
        return talon;
    }
    // public static WPI_TalonFX createPermanentSimulationFollowerTalon(int follower_id, int leader_id, String canBus) {
    //     final WPI_TalonFX talon = createSimulationTalon(follower_id, canBus, kFollowerConfiguration);
    //     talon.set(ControlMode.Follower, leader_id);
    //     return talon;
    // }

    public static TalonFX createTalon(int id, String canBus, TalonFXConfiguration config) {
        TalonFX talon = new TalonFX(id, canBus); // TODO Use LazyTalonFX here once it's fixed

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();


        talon.clearStickyFaults(kTimeoutSeconds);
        talon.setPosition(0);
        
        motorConfig = config;
        

        // talon.optimizeBusUtilization();
        ErrorCheckUtil.checkError(talon.getConfigurator().apply(motorConfig), CommonErrorNames.ConfiguringTalon(id));

        return talon;
    }
    // public static WPI_TalonFX createSimulationTalon(int id, String canBus, Configuration config) {
    //     WPI_TalonFX talon = new WPI_TalonFX(id, canBus);
    //     talon.set(ControlMode.PercentOutput, 0.0);

    //     talon.changeMotionControlFramePeriod(1000);
    //     talon.clearMotionProfileHasUnderrun(kTimeoutMs);
    //     talon.clearMotionProfileTrajectories();

    //     talon.clearStickyFaults(kTimeoutMs);

    //     return talon;
    // }
}