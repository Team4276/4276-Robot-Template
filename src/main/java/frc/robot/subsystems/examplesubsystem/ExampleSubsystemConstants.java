package frc.robot.subsystems.examplesubsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;

import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ExampleSubsystemConstants {
    public static final Voltage kExampleVoltage = Units.Volts.of(5.0);
    public static final Voltage kIdleVoltage = Units.Volts.of(0.0);

    public static final double kGearing = 1.0;

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 50.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.Feedback.SensorToMechanismRatio = kGearing;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    /** Config helper for a custom motor CAN ID and CANBus object */
    public static MotorIOTalonFXConfig getIOConfig(int id, CANBus bus) {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.unit = Units.Rotations;
        config.time = Units.Minutes;
        config.mainID = id;
        config.mainBus = bus;
        config.mainConfig = getFXConfig();
        return config;
    }

    /** Config helper for a custom motor CAN ID and CANBus string (e.g., "canivore1") */
    public static MotorIOTalonFXConfig getIOConfig(int id, String busName) {
        return getIOConfig(id, new CANBus(busName));
    }

    /** Default config helper using Ports.EXAMPLE_SUBSYSTEM */
    public static MotorIOTalonFXConfig getIOConfig() {
        return getIOConfig(Ports.EXAMPLE_SUBSYSTEM.id, Ports.EXAMPLE_SUBSYSTEM.bus);
    }

    /** Get MotorIO instance for a custom motor CAN ID and CANBus */
    public static MotorIO getMotorIO(int id, CANBus bus) {
        return switch (RobotConstants.mode) {
            case REAL -> new MotorIOTalonFX(getIOConfig(id, bus));
            case SIM -> new MotorIOTalonFXSim(getIOConfig(id, bus), new RollerSim(getSimConstants()));
            case REPLAY -> new MotorIO(Units.Rotations, Units.Minutes) {
                @Override
                public void updateInputs() {}
            };
        };
    }

    /** Default MotorIO getter */
    public static MotorIO getMotorIO() {
        return getMotorIO(Ports.EXAMPLE_SUBSYSTEM.id, Ports.EXAMPLE_SUBSYSTEM.bus);
    }

    public static RollerSimConstants getSimConstants() {
        RollerSimConstants simConstants = new RollerSimConstants();

        simConstants.motor = DCMotor.getKrakenX60(1);
        simConstants.gearing = kGearing;
        simConstants.momentOfInertia = 0.01;

        return simConstants;
    }
}