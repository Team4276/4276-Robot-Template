package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.LinearSim;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.util.Util;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ElevatorConstants {
    // ============ HARDWARE PLACEHOLDERS — fill in real values ============
    // CAN ID & gear ratio come from electrical/mechanical design.
    public static final double kGearing = 1.0;
    // Elevator height travel in meters (for the sim).
    public static final Distance kMinHeight = Units.Meters.of(0.0);
    public static final Distance kMaxHeight = Units.Meters.of(1.0);
    // Diameter of the drum the cable/spool wraps around (for sim + position conversion).
    public static final Distance kDrumDiameter = Units.Meters.of(0.05);
    public static final Mass kCarriageMass = Units.Kilograms.of(5.0);

    // Which way the follower motor should spin relative to the leader.
    // Aligned: same direction as leader. Opposed: opposite direction (typical for
    // two elevator motors driving the same drum from mirrored mounting).
    public static final MotorAlignmentValue kFollowerAlignment = MotorAlignmentValue.Opposed;

    // Position setpoints, in rotations of the mechanism (post gearing).
    public static final Angle kHomePosition = Units.Rotations.of(0.0);
    public static final Angle kLevel1Position = Units.Rotations.of(5.0);
    public static final Angle kLevel2Position = Units.Rotations.of(10.0);
    public static final Angle kLevel3Position = Units.Rotations.of(15.0);

    public static final Angle kEpsilonThreshold = Units.Rotations.of(0.02);
    // ============ END HARDWARE PLACEHOLDERS ============

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
        config.mainID = Ports.ELEVATOR.id;
        config.mainBus = Ports.ELEVATOR.bus;

        config.followerIDs = new int[] { Ports.ELEVATOR_FOLLOWER.id };
        config.followerBuses = new CANBus[] { Ports.ELEVATOR_FOLLOWER.bus };
        config.followerAlignment = new MotorAlignmentValue[] { kFollowerAlignment };

        return config;
    }

    public static MotorIO getMotorIO() {
        return switch (RobotConstants.mode) {
            case REAL -> new MotorIOTalonFX(getIOConfig());
            case SIM -> new MotorIOTalonFXSim(getIOConfig(), new LinearSim(getSimConstants()));
            case REPLAY -> new MotorIO(Units.Rotations, Units.Minutes) {
                @Override
                public void updateInputs() {
                };
            };
        };
    }

    public static LinearSimConstants getSimConstants() {
        LinearSimConstants simConstants = new LinearSimConstants();

        Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(
                kDrumDiameter.div(2.0));

        simConstants.converter = converter;
        simConstants.motor = DCMotor.getKrakenX60(1);
        simConstants.gearing = kGearing;
        simConstants.carriageMass = kCarriageMass;
        simConstants.minHeight = kMinHeight;
        simConstants.maxHeight = kMaxHeight;
        simConstants.simGravity = true;
        simConstants.startingHeight = converter
                .toDistance(kHomePosition);

        return simConstants;
    }

    public static ServoHomingConfig getServoHomingConfig() {
        ServoHomingConfig config = new ServoHomingConfig();
        config.kHomePosition = Units.Rotations.of(0.0);
        config.kHomingVoltage = Units.Volts.of(-1.0);
        config.kHomingTimeout = Units.Seconds.of(0.25);
        config.kSetHomedVelocity = Units.RotationsPerSecond.of(0.05);
        return config;
    }
}
