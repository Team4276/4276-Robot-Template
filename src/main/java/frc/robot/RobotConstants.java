package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotConstants {
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,
    }

    public static Mode getMode() {
        return mode;
    }

    public static enum RobotType {
        COMPBOT,
        SIMBOT
    }

    public static Mode mode = Mode.SIM;

    public static RobotType getType() {
        return switch (mode) {
            case REAL -> RobotType.COMPBOT;
            case REPLAY -> RobotType.COMPBOT;
            case SIM -> RobotType.SIMBOT;
        };
    }
    
	public static final CANBus canivore1 = new CANBus("canivore1");
	public static final CANBus rio = new CANBus();

	public static Sendable LOGGED_SENDABLES[] = new Sendable[] {
		Drive.mInstance,
        ExampleSubsystem.mInstance,
		Superstructure.mInstance
	};
}
