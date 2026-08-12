package frc.robot.subsystems.examplesubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.Ports;
import frc.robot.Robot;

public class ExampleSubsystemConstants {
	public static final Voltage kExampleVoltage = Units.Volts.of(0.0);
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

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.unit = Units.Rotations;
		config.time = Units.Minutes;
		config.mainID = Ports.EXAMPLE_SUBSYSTEM.id;
		config.mainBus = Ports.EXAMPLE_SUBSYSTEM.bus;
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		}
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.motor = DCMotor.getKrakenX60(1);
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = 0.01;

		return simConstants;
	}
}
