package frc.robot.controlboard;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.ControllerUtil;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;

public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	private CommandXboxController driver = new CommandXboxController(ControlBoardConstants.kDriverControllerPort);
	private CommandXboxController operator = new CommandXboxController(ControlBoardConstants.kOperatorControllerPort);

	public void configureBindings() {
		// Drive.mInstance.setDefaultCommand(Drive.mInstance.followSwerveRequestCommand(
		// DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater));
		// driver.back()
		// .onTrue(Commands.runOnce(
		// () -> Drive.mInstance.getGeneratedDrive().seedFieldCentric(),
		// Drive.mInstance)
		// .ignoringDisable(true));

		driver.start()
				.onTrue(Commands.runOnce(() -> Robot.resetPoseForAuto = true).ignoringDisable(true));

		driverControls();
		// bringupControls();
		// jogControls();
		// tuningControls();
	}

	public void driverControls() {
		driver.a()
				.onTrue(Superstructure.mInstance.exampleCommand().onlyWhile(driver.a()));
		driver.b()
				.onTrue(Superstructure.mInstance.testCommand().onlyWhile(driver.b()));
	}

	public void bringupControls() {
		driver.a().onTrue(ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT));
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
				operator.a(),
				operator.b(),
				operator.rightBumper());
	}

	public void tuningControls() {
	}

	public Command rumbleCommand(Time duration) {
		return rumbleCommand(driver, duration);
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
		setRumble(driver, on);
	}

	public void setRumble(CommandXboxController controller, boolean on) {
		controller.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}
}
