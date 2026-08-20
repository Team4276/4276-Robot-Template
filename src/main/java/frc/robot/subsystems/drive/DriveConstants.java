package frc.robot.subsystems.drive;

import frc.robot.controlboard.ControlBoard;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class DriveConstants {
	public static final SwerveInputStream kTeleopAngularVelocityStream = Drive.mInstance.getAngularVelocityStream(ControlBoard.mDriver::getLeftY,
			ControlBoard.mDriver::getLeftX,
			ControlBoard.mDriver::getRightX)
			.withAllianceRelativeControl();
}
