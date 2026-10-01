package frc.robot.subsystems.examplesubsystem;

import com.ctre.phoenix6.CANBus;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

public class ExampleSubsystem extends MotorSubsystem<MotorIO> {
    public static final Setpoint EXAMPLE_SETPOINT = Setpoint
            .withVoltageSetpoint(ExampleSubsystemConstants.kExampleVoltage);
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(ExampleSubsystemConstants.kIdleVoltage);

    public static final ExampleSubsystem mInstance = new ExampleSubsystem();

    /** Default constructor using default constants */
    public ExampleSubsystem() {
        super(ExampleSubsystemConstants.getMotorIO(), "Example Subsystem");
    }

    /** Constructor for testing a specific motor by CAN ID and CANBus object */
    public ExampleSubsystem(int motorId, CANBus canBus) {
        super(ExampleSubsystemConstants.getMotorIO(motorId, canBus), "Kraken Test ID " + motorId);
    }

    /** Constructor for testing a specific motor by CAN ID and CANBus string (e.g. "canivore1") */
    public ExampleSubsystem(int motorId, String canBusName) {
        this(motorId, new CANBus(canBusName));
    }

    /** Apply a voltage once using setpointCommand */
    public Command runVoltageOnceCommand(Voltage voltage) {
        return setpointCommand(Setpoint.withVoltageSetpoint(voltage));
    }

    /** Run a target voltage continuously using followSetpointCommand */
    public Command followVoltageCommand(Voltage voltage) {
        return followSetpointCommand(() -> Setpoint.withVoltageSetpoint(voltage));
    }

    /** Apply test voltage while active and return to neutral when command ends */
    public Command testVoltageCommand(Voltage voltage) {
        return runEnd(
            () -> applySetpoint(Setpoint.withVoltageSetpoint(voltage)),
            () -> applySetpoint(IDLE)
        ).withName("Test Voltage Command");
    }

    /** Maps live joystick input (-1.0 to 1.0) to motor voltage (-maxVoltage to +maxVoltage) */
    public Command runVoltageWithJoystick(DoubleSupplier joystickSupplier, double maxVoltage) {
        return followSetpointCommand(() -> {
            double volts = joystickSupplier.getAsDouble() * maxVoltage;
            return Setpoint.withVoltageSetpoint(Units.Volts.of(volts));
        }).withName("Joystick Voltage Control");
    }
}