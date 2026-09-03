package frc.robot.controlboard;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.hid.ViXController;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.ControllerUtil;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.AimAtCommand;
import frc.robot.game.FieldLayout;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;

public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	public static final ViXController mDriver = new ViXController(
			ControlBoardConstants.kDriverControllerPort);
	public static final ViXController mOperator = new ViXController(
			ControlBoardConstants.kOperatorControllerPort);
	public static final CommandGenericHID mKeyboard0 = new CommandGenericHID(0);
	public static final CommandGenericHID mKeyboard1 = new CommandGenericHID(1);

	public void configureBindings() {

		Drive.mInstance.setDefaultCommand(Drive.mInstance.drive(DriveConstants.kTeleopRequestUpdater));

		mDriver.back()
				.onTrue(Commands.runOnce(
						() -> Drive.mInstance.zeroGyro(),
						Drive.mInstance)
						.ignoringDisable(true));

		mDriver.start()
				.onTrue(Commands.runOnce(() -> Robot.resetPoseForAuto = true).ignoringDisable(true));

		driverControls();
		// bringupControls();
		// jogControls();
		// tuningControls();

		if (RobotConstants.getMode() == RobotConstants.Mode.SIM) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
	}

	public void driverControls() {
		mKeyboard0.button(1)
				.onTrue(Superstructure.mInstance.exampleCommand().onlyWhile(mKeyboard0.button(1)));
		mKeyboard0.button(2)
				.onTrue(Superstructure.mInstance.testCommand().onlyWhile(mKeyboard0.button(2)));

		mKeyboard0.button(3)
				.whileTrue(new AimAtCommand(() -> {
					Translation2d poi = FieldLayout.kPOI;
					Pose2d pose = Drive.mInstance.getPose();
					return new Rotation2d(
							poi.getX() - pose.getX(),
							poi.getY() - pose.getY());
				}));
	}

	public void bringupControls() {
		mDriver.a().onTrue(ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT));
	}

	public void jogControls() {
		ControllerUtil.bindJog(
				ExampleSubsystem.mInstance,
				Volts.of(0.01).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				mOperator.a(),
				mOperator.b(),
				mOperator.rightBumper());
	}

	public void tuningControls() {
	}

	public Command rumbleCommand(Time duration) {
		return rumbleCommand(mDriver, duration);
	}

	public Command rumbleCommand(CommandXboxController controller, Time duration) {
		return Commands.sequence(
				Commands.runOnce(() -> {
					setRumble(controller, true);
				}),
				Commands.waitSeconds(duration.in(Units.Seconds)),
				Commands.runOnce(() -> {
					setRumble(controller, false);
				}))
				.finallyDo(() -> {
					setRumble(controller, false);
					;
				})
				.withName("Rumble");
	}

	public void setRumble(boolean on) {
		setRumble(mDriver, on);
	}

	public void setRumble(CommandXboxController controller, boolean on) {
		controller.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}
}
