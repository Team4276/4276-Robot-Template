package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;

public class Superstructure extends SubsystemBase {
    public static final Superstructure mInstance = new Superstructure();

    public Command exampleCommand() {
        return ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT)
                .handleInterrupt(() -> {
                    ExampleSubsystem.mInstance.applySetpoint(ExampleSubsystem.EXAMPLE_SETPOINT);
                })
                .withName("Example Command");
    }
}
