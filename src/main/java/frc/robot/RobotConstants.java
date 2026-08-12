package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotConstants {
    
	public static final CANBus canivore1 = new CANBus("canivore1");
	public static final CANBus rio = new CANBus();

	public static Sendable LOGGED_SENDABLES[] = new Sendable[] {
		Drive.mInstance,
        ExampleSubsystem.mInstance,
		Superstructure.mInstance
	};
}
