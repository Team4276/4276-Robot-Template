package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * Aims the robot's heading at a given target yaw and holds it, while leaving
 * translation under the driver's control.
 */
public class AimAtCommand extends Command {
    private final Supplier<Rotation2d> targetYaw;

    public AimAtCommand(Supplier<Rotation2d> targetYaw) {
        this.targetYaw = targetYaw;
        addRequirements(Drive.mInstance);
    }

    @Override
    public void initialize() {
        DriveConstants.kHeadingAlignController.reset();
    }

    @Override
    public void execute() {
        ChassisSpeeds translation = DriveConstants.kTeleopRequestUpdater.get();

        double error = MathUtil.angleModulus(
                AllianceFlipUtil.apply(targetYaw.get())
                        .minus(Drive.mInstance.getPose().getRotation())
                        .getRadians());

        double omega = DriveConstants.kHeadingAlignController.calculate(0.0, error);

        Logger.recordOutput("AimAtCommand/TargetYaw", targetYaw.get().getRadians());
        Logger.recordOutput("AimAtCommand/Error", error);
        Logger.recordOutput("AimAtCommand/Omega", omega);

        Drive.mInstance.driveFieldRelative(
                new ChassisSpeeds(translation.vxMetersPerSecond, translation.vyMetersPerSecond, omega));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
