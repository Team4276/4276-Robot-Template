package frc.robot.subsystems.drive;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class DriveIOYAGSL implements DriveIO {
    private SwerveDrive mSwerveDrive;

    public DriveIOYAGSL(){
        var cfg = new SwerveDriveConfig()
                .withStartingPose(new Pose2d(3.0, 3.0, Rotation2d.kZero))
                .withTelemetry(TelemetryVerbosity.HIGH);
        try {
            mSwerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/base"))
                    .createSwerveDrive(cfg);
        } catch (Exception e) {
            System.out.println("Error creating swerve drive");
            System.out.println(e);
            throw new RuntimeException(e);
        }
    }
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        mSwerveDrive.updateTelemetry();

        inputs.pose = mSwerveDrive.getPose();
        inputs.gyroAngle = mSwerveDrive.getGyroAngle();
        inputs.fieldRelativeSpeed = mSwerveDrive.getFieldRelativeSpeed();
        inputs.robotRelativeSpeed = mSwerveDrive.getRobotRelativeSpeed();
        
        inputs.modulesPositions = mSwerveDrive.getModulePositions();
        inputs.moduleStates = mSwerveDrive.getModuleStates();

        // Ha who needs exception handling
        inputs.moduleInputs[0] = getFromModule(mSwerveDrive.getModule("frontleft").orElseThrow());
        inputs.moduleInputs[1] = getFromModule(mSwerveDrive.getModule("frontright").orElseThrow());
        inputs.moduleInputs[2] = getFromModule(mSwerveDrive.getModule("backleft").orElseThrow());
        inputs.moduleInputs[3] = getFromModule(mSwerveDrive.getModule("backright").orElseThrow());
    }

    private ModuleInput getFromModule(SwerveModule module){
        return new ModuleInput(
            true, 
            module.getDriveMotorController().getRotorPosition(), 
            module.getDriveMotorController().getRotorVelocity(), 
            module.getDriveMotorController().getVoltage(), 
            module.getDriveMotorController().getSupplyCurrent().orElse(Units.Amps.of(0)), 
            module.getDriveMotorController().getStatorCurrent(), 
            module.getDriveMotorController().getTemperature(), 
            true,
            module.getAzimuthMotorController().getMechanismPosition(), 
            module.getAzimuthMotorController().getMechanismVelocity(), 
            module.getAzimuthMotorController().getVoltage(), 
            module.getAzimuthMotorController().getSupplyCurrent().orElse(Units.Amps.of(0)), 
            module.getAzimuthMotorController().getStatorCurrent(), 
            module.getAzimuthMotorController().getTemperature());
    }

    @Override
    public void updateSim(){
        mSwerveDrive.simIterate();
    }

    @Override
    public void resetPose(Pose2d pose) {
        mSwerveDrive.resetOdometry(pose);
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        mSwerveDrive.setFieldRelativeChassisSpeeds(speeds);
    }

    
}
