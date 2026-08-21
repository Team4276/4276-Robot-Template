package frc.robot.subsystems.drive;

import frc.robot.controlboard.ControlBoard;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class DriveConstants {
	// public static final SwerveInputStream kTeleopAngularVelocityStream = Drive.mInstance.getAngularVelocityStream(
	// 		ControlBoard.mDriver::getLeftY,
	// 		ControlBoard.mDriver::getLeftX,
	// 		ControlBoard.mDriver::getRightX)
	// 		.withAllianceRelativeControl();

	public static final SwerveInputStream kTeleopAngularVelocityStream = Drive.mInstance.getAngularVelocityStream(
			() -> ControlBoard.mKeyboard0.getRawAxis(1),
			() -> ControlBoard.mKeyboard0.getRawAxis(0),
			() -> -ControlBoard.mKeyboard1.getRawAxis(0))
			.withAllianceRelativeControl();
}
