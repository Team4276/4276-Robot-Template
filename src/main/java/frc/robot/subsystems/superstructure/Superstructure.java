package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;

public class Superstructure extends SubsystemBase {
    public static final Superstructure mInstance = new Superstructure();

    @Override
    public void periodic() {

    }

    public Command exampleCommand() { // TODO: check if works on button hold
        return ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT)
                .finallyDo(() -> {
                    ExampleSubsystem.mInstance.applySetpoint(ExampleSubsystem.IDLE);
                })
                .withName("Example Command");
    }

    public Command testCommand() {
        return Commands.runEnd(() -> {
            Logger.recordOutput("Superstructure/Test", true);
        },
                () -> {
                    Logger.recordOutput("Superstructure/Test", false);
                })
                .withName("Test Command");
    }
}
