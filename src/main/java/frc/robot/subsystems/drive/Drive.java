package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AllianceFlipUtil;

public class Drive extends SubsystemBase {
    public static final Drive mInstance = new Drive();

    private DriveIO io = new DriveIOYAGSL();
    private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private Drive() {
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }

    public Pose2d getPose() {
        return inputs.pose;
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    public void zeroGyro() {
        resetPose(
                new Pose2d(
                        getPose().getTranslation(),
                        AllianceFlipUtil.apply(Rotation2d.kZero)));

    }

    public Command drive(ChassisSpeeds speeds) {
        return drive(() -> speeds);
    }

    public Command drive(Supplier<ChassisSpeeds> speeds) {
        return Commands.run(() -> io.drive(speeds.get()), this);
    }

    public void followChoreoTrajectory(SwerveSample sample) {
        ChassisSpeeds requestedSpeeds = sample.getChassisSpeeds();

        requestedSpeeds.vxMetersPerSecond += DriveConstants.kTrajectoryXController.calculate(
                0.0, sample.x - getPose().getTranslation().getX());
        requestedSpeeds.vyMetersPerSecond += DriveConstants.kTrajectoryYController.calculate(
                0.0, sample.y - getPose().getTranslation().getY());
        requestedSpeeds.omegaRadiansPerSecond += DriveConstants.kTrajectoryThetaController.calculate(
                0.0,
                MathUtil.angleModulus(
                        sample.getPose()
                                .getRotation()
                                .minus(getPose().getRotation())
                                .getRadians()));

        Logger.recordOutput("Drive/Trajectory/SetpointPose", sample.getPose());
        Logger.recordOutput(
                "Drive/Trajectory/SetpointSpeeds", sample.getChassisSpeeds());

        io.drive(requestedSpeeds);
    }
}
