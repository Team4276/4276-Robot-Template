package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.lib.util.LoggedTunablePID;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoardConstants;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class DriveConstants {
	public static final SwerveInputStream kTeleopAngularVelocityStream = switch (ControlBoardConstants.kInputMode) {
		case CONTROLLER -> Drive.mInstance.getAngularVelocityStream(
				() -> ControlBoard.mDriver.getLeft().sq().y,
				() -> ControlBoard.mDriver.getLeft().sq().x,
				() -> ControlBoard.mDriver.getRight().sq().x)
				.withAllianceRelativeControl();
		case KEYBOARD -> Drive.mInstance.getAngularVelocityStream(
				() -> ControlBoard.mKeyboard0.getRawAxis(1),
				() -> ControlBoard.mKeyboard0.getRawAxis(0),
				() -> -ControlBoard.mKeyboard1.getRawAxis(0))
				.withAllianceRelativeControl();
		case DEMO -> Drive.mInstance.getAngularVelocityStream(
				() -> ControlBoard.mDriver.getLeft().sq().y,
				() -> ControlBoard.mDriver.getLeft().sq().x,
				() -> ControlBoard.mDriver.getRight().sq().x)
				.withAllianceRelativeControl();
	};

	public static final LoggedTunablePID kTeleopAutoAlignController = new LoggedTunablePID(
			3.0, 0, 0.1, Units.inchesToMeters(1.0), "Drive/AutoAlign/TeleopTranslation");
	public static final LoggedTunablePID kHeadingAlignController = new LoggedTunablePID(20.0, 0, 0, Math.toRadians(1.0),
			"Drive/HeadingAlign");

	public static final LoggedTunablePID kTrajectoryXController = new LoggedTunablePID(5.0, 0, 0,
			Units.inchesToMeters(0.5),
			"Drive/Trajectory/Translation");
	public static final LoggedTunablePID kTrajectoryYController = new LoggedTunablePID(5.0, 0, 0,
			Units.inchesToMeters(0.5),
			"Drive/Trajectory/Translation");
	public static final LoggedTunablePID kTrajectoryThetaController = new LoggedTunablePID(3.0, 0, 0,
			Math.toRadians(1.0),
			"Drive/Trajectory/Rotation");
}
