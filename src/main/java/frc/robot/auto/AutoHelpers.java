package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoHelpers {
    // Add event markers and common commands/helpers here

    public static Command exampleCommand() {
        return Superstructure.mInstance.exampleCommand();
    }
}
