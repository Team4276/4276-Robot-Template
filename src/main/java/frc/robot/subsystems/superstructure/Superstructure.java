package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;

public class Superstructure extends SubsystemBase {
    public static final Superstructure mInstance = new Superstructure();

    @Override
    public void periodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    public Command exampleCommand() {
        return ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT)
                .handleInterrupt(() -> {
                    ExampleSubsystem.mInstance.applySetpoint(ExampleSubsystem.IDLE);
                })
                .withName("Example Command");
    }

    public Command testCommand() {
        return Commands.runOnce(() -> {
            Logger.recordOutput("Superstructure/Test", true);
        })
                .handleInterrupt(() -> {
                    Logger.recordOutput("Superstructure/Test", false);
                })
                .withName("Test Command");
    }
}
