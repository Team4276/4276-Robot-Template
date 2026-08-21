package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.AllianceFlipUtil;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Drive extends SubsystemBase {
    public static final Drive mInstance = new Drive();

    private SwerveDrive mSwerveDrive;

    private Drive() {
        SmartDashboard.putData(this);
        var cfg = new SwerveDriveConfig()
                .withStartingPose(new Pose2d(3.0, 3.0, Rotation2d.kZero))
                .withSubsystem(this)
                .withTelemetry(TelemetryVerbosity.HIGH);
        try {
            mSwerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/base"))
                    .createSwerveDrive(cfg);
        } catch (Exception e) {
            System.out.println("Error creating swerve drive");
            System.out.println(e);
            throw new RuntimeException(e);
        }

        // TODO Vision
        mSwerveDrive.addVisionMeasurement(getPose(), 0);
    }

    @Override
    public void periodic() {
        mSwerveDrive.updateTelemetry();

        // TODO Add logging for all fields in high verbosity (replay not possible on
        // hardware level ;-;)
        // I mean it won't be possible when we have ctre either sooo
    }

    @Override
    public void simulationPeriodic() {
        mSwerveDrive.simIterate();
    }

    // TODO this doesn't work btw... need to do this through an io getter which
    // means i have to implement a drive io which means moving all of this code
    // elsewhere...
    @AutoLogOutput(key = "Drive/Pose")
    public Pose2d getPose() {
        return mSwerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        mSwerveDrive.resetOdometry(pose);
    }

    public void zeroGyro() {
        resetPose(
                new Pose2d(
                        getPose().getTranslation(),
                        AllianceFlipUtil.apply(Rotation2d.kZero)));

    }

    public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return new SwerveInputStream(mSwerveDrive, x, y, rot);
    }

    public Command drive(ChassisSpeeds speeds) {
        return mSwerveDrive
                .drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                        new Rotation2d(mSwerveDrive.getGyroAngle())));
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

        mSwerveDrive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(requestedSpeeds,
                new Rotation2d(mSwerveDrive.getGyroAngle())));
    }

    /**
     * Create a {@link Command} that runs a full SysId characterization routine
     * (quasistatic and dynamic, forward and
     * reverse) on a single swerve module's drive motor. The module's azimuth is
     * held pointed straight ahead for the
     * duration of the test so only the drive motor is characterized.
     *
     * @param moduleName Name of the module to test, e.g. "frontleft", "frontright",
     *                   "backleft", or "backright".
     * @return {@link Command} that runs the full SysId routine on the given module.
     */
    public Command sysIdModule(String moduleName) {

        SwerveModule module = mSwerveDrive.getModule(moduleName).orElseThrow();
        SmartMotorController driveMotor = module.getDriveMotorController();
        SmartMotorController azimuthMotor = module.getAzimuthMotorController();

        SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        azimuthMotor::setVoltage,
                        log -> log.motor(moduleName + "-azimuth")
                                .voltage(azimuthMotor.getVoltage())
                                .angularPosition(azimuthMotor.getMechanismPosition())
                                .angularVelocity(azimuthMotor.getMechanismVelocity()),
                        this,
                        moduleName + "-azimuth"));

        return Commands.runOnce(() -> azimuthMotor.setPosition(Rotation2d.kZero.getMeasure()))
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                .andThen(Commands.waitSeconds(1))
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                .andThen(Commands.waitSeconds(1))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                .andThen(Commands.waitSeconds(1))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                .withName("SysId " + moduleName + " Azimuth");
    }
}
