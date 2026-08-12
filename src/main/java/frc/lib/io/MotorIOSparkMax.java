package frc.lib.io;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/**
 * Tuning and setpoint with config change requests no work
 */
public class MotorIOSparkMax extends MotorIO {
	protected final SparkMax main;
	protected final SparkMax[] followers;
	protected SparkMaxConfig config;
	protected SparkMaxConfig followerConfig;
	private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
	private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5,
			java.util.concurrent.TimeUnit.MILLISECONDS, queue);
	private boolean configFailed = false;

	public void applyConfig(SparkMax spark, SparkMaxConfig config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				REVLibError result = spark.configure(config, ResetMode.kResetSafeParameters,
						PersistMode.kPersistParameters);
				if (result == REVLibError.kOk) {
					break;
				} else {
					configFailed = true;
				}
			}
		});
	}

	@Override
	public void updateInputs() {
		updateMotorInputs(inputs, main);

		for (int i = 0; i < followers.length; i++) {
			updateMotorInputs(followerInputs[i], followers[i]);
		}
	}

	/**
	 * Updates one Inputs from readings of a TalonFX motor
	 *
	 * @param inputsToUpdate Inputs to update from reading.
	 * @param motor          Motor to read from.
	 */
	protected void updateMotorInputs(Inputs inputsToUpdate, SparkMax motor) {
		inputsToUpdate.position = Units.Rotations.of(motor.getEncoder().getPosition());
		inputsToUpdate.velocity = Units.RotationsPerSecond.of(motor.getEncoder().getVelocity());
		inputsToUpdate.statorCurrent = Units.Amps.of(motor.getOutputCurrent());
		inputsToUpdate.supplyCurrent = Units.Amps.of(motor.getOutputCurrent());
		inputsToUpdate.motorVoltage = Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
		inputsToUpdate.pidVoltage = Units.Volts.of(0.0);
		inputsToUpdate.motorTemperature = Units.Celsius.of(motor.getMotorTemperature());
		inputsToUpdate.acceleration = Units.RotationsPerSecondPerSecond.of(0.0);
	}

	@Override
	public void setNeutralSetpoint() {
		main.stopMotor();
	}

	@Override
	public void setCoastSetpoint() {
		threadPoolExecutor.submit(() -> {
			main.configure(config.idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		});
	}

	@Override
	protected void setVoltageSetpoint(Voltage voltage) {
		main.setVoltage(voltage);
	}

	@Override
	protected void setDutyCycleSetpoint(Dimensionless percent) {
		main.set(percent.in(Percent));
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition) {
		setMotionMagicSetpoint(mechanismPosition, 0);
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition, int slot) {
		main.getClosedLoopController().setSetpoint(mechanismPosition.in(Rotations), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.fromInt(slot));
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		setVelocitySetpoint(mechanismVelocity, 1);
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity, int slot) {
		main.getClosedLoopController().setSetpoint(mechanismVelocity.in(RotationsPerSecond), ControlType.kVelocity, ClosedLoopSlot.fromInt(slot));
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition) {
		setPositionSetpoint(mechanismPosition, 2);
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition, int slot) {
		main.getClosedLoopController().setSetpoint(mechanismPosition.in(Rotations), ControlType.kPosition, ClosedLoopSlot.fromInt(slot));
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			main.getEncoder().setPosition(mechanismPosition.in(Rotations));
		});
	}

	@Override
	public void zeroSensors() {
		setCurrentPosition(Units.Rotations.of(0.0));
	}

	private void setIdleMode(SparkMax spark, IdleMode idleMode) {
		SmartDashboard.putNumber("SPARK MAX NEUTRAL MODE SET!!", Timer.getFPGATimestamp());
		threadPoolExecutor.submit(() -> {
			main.configure(config.idleMode(idleMode), ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		});
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		IdleMode idleMode = wantsBrake ? IdleMode.kBrake : IdleMode.kCoast;
		setIdleMode(main, idleMode);
		for (SparkMax spark : followers) {
			setIdleMode(spark, idleMode);
		}
	}

	@Override
	public void useSoftLimits(boolean enable) {
		// hahahahahaha hahahahaHAha hahaHAhaha hahaha ha
	}

	@Override
	public TalonFXConfiguration getMotorIOConfig() {
		// hahahahahaha hahahahaHAha hahaHAhaha hahaha ha
		return new TalonFXConfiguration();
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * Applies a SparkMaxConfig to the main motor.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setMainConfig(SparkMaxConfig configuration) {
		config = configuration;
		applyConfig(main, config);
	}

	public void setMainConfig(TalonFXConfiguration configuration) {
		// hahahahahaha hahahahaHAha hahaHAhaha hahaha ha
	}

	/**
	 * Changes the currently applied main TalonFXConfiguration and applies the new
	 * configuration to the main motor.
	 *
	 * @param configChanger Mutating operation to apply on the current
	 *                      configuration.
	 */
	public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		// hahahahahaha hahahahaHAha hahaHAhaha hahaha ha
	}

	/**
	 * Changes the currently applied follower TalonFXConfiguration and applies the
	 * new configuration to all follower motors.
	 *
	 * @param configChanger Mutating operation to apply on the current
	 *                      configuration.
	 */
	public void changeFollowerConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		// hahahahahaha hahahahaHAha hahaHAhaha hahaha ha
	}

	/**
	 * Creates a MotorIOTalonFX from a provided configuration.
	 *
	 * @param config Configuration to create MotorIOTalonFX from.
	 */
	public MotorIOSparkMax(MotorIOSparkMaxConfig config) {
		super(config.unit, config.time, config.followerIDs.length);
		main = new SparkMax(config.mainID, MotorType.kBrushless);
		setMainConfig(config.mainConfig);

		followers = new SparkMax[config.followerIDs.length];
		for (int i = 0; i < config.followerIDs.length; i++) {
			followers[i] = new SparkMax(config.followerIDs[i], MotorType.kBrushless);
			followerConfig.follow(main, config.followerInverted[i]);
			applyConfig(followers[i], followerConfig);
		}
	}

	/**
	 * Configuration for a MotorIOTalonFX. Motion magic control is on slot 0,
	 * velocity on slot 1, and position PID on slot 2.
	 */
	public static class MotorIOSparkMaxConfig {
		public AngleUnit unit = Units.Rotations;
		public TimeUnit time = Units.Seconds;
		public int mainID = -1;
		public SparkMaxConfig mainConfig = new SparkMaxConfig();
		public int[] followerIDs = new int[0];
		public SparkMaxConfig followerConfig = new SparkMaxConfig();
		public boolean[] followerInverted = new boolean[0];
	}

	@Override
	public boolean getConfigFailed() {
		return configFailed;
	}

	public SparkMax getMain() {
		return main;
	}
}
