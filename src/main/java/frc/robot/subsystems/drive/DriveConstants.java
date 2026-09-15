package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.LoggedTunablePID;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoardConstants;

public class DriveConstants {
	public static final LinearVelocity kMaxVelocity = org.wpilib.units.Units.MetersPerSecond.of(5.623);
	public static final AngularVelocity kMaxOmega = org.wpilib.units.Units.RadiansPerSecond.of(13.154);

	private static ChassisVelocities getRequestedSpeeds(double xInput, double yInput, double rotationInput) {
		double linearMagnitude = Math.hypot(xInput, yInput);

		// Square magnitude for more precise control
		linearMagnitude = linearMagnitude * linearMagnitude;

		Translation2d linearVelocity = Translation2d.kZero;

		if (linearMagnitude > 1e-6) {
			linearVelocity = new Translation2d(
					linearMagnitude,
					new Rotation2d(
							xInput, yInput))
					.times(DriveConstants.kMaxVelocity.baseUnitMagnitude());
		}

		// Square rotation value for more precise control
		double omega = Math.copySign(
				rotationInput * rotationInput,
				-rotationInput);

		return new ChassisVelocities(
						linearVelocity.getX(),
						linearVelocity.getY(),
						omega * DriveConstants.kMaxOmega.baseUnitMagnitude()).toRobotRelative(AllianceFlipUtil.apply(Rotation2d.k180deg));
	}

	public static final Supplier<ChassisVelocities> kTeleopRequestUpdater = switch (ControlBoardConstants.kInputMode) {
		case CONTROLLER -> () -> {
			return getRequestedSpeeds(
					ControlBoard.mDriver.getLeftWithDeadband().y,
					ControlBoard.mDriver.getLeftWithDeadband().x,
					ControlBoard.mDriver.getRightWithDeadband().x);
		};
		case KEYBOARD -> () -> {
			return getRequestedSpeeds(
					ControlBoard.mKeyboard0.getRawAxis(1),
					ControlBoard.mKeyboard0.getRawAxis(0),
					ControlBoard.mKeyboard1.getRawAxis(0));
		};
		case DEMO -> () -> {
			return getRequestedSpeeds(
					ControlBoard.mDriver.getLeftWithDeadband().y,
					ControlBoard.mDriver.getLeftWithDeadband().x,
					ControlBoard.mDriver.getRightWithDeadband().x);
		};

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
