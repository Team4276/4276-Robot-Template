package frc.robot.controlboard;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;

public class ControlBoardConstants {
	public static final int kDriverControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

	public static final Time kIntakeRumbleTime = Units.Seconds.of(0.2);

	public static final double kStickDeadband = 0.05;
}
