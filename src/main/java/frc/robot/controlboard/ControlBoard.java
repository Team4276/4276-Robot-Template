package frc.robot.controlboard;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;

public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	private CommandXboxController driver = new CommandXboxController(ControlBoardConstants.kDriverControllerPort);
	private CommandXboxController operator = new CommandXboxController(ControlBoardConstants.kOperatorControllerPort);

	public void configureBindings() {
		driver.start()
				.onTrue(Commands.runOnce(() -> Robot.resetPoseForAuto = true).ignoringDisable(true));

		driverControls();
		// bringupControls();
		// jogControls();
		// tuningControls();
	}

	public void driverControls() {
		// Competition controls
	}

	public void bringupControls() {
	}

	public void jogControls() {
	}

	public void tuningControls() {
	}

	public Command rumbleCommand(Time duration){
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

	public void setRumble(boolean on){
		setRumble(driver, on);
	}

	public void setRumble(CommandXboxController controller, boolean on) {
		controller.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}
}
