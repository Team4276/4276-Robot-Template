package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        Pose2d pose = Pose2d.kZero;
        Angle gyroAngle = Units.Degrees.of(0);
        ChassisSpeeds fieldRelativeSpeed = new ChassisSpeeds();
        ChassisSpeeds robotRelativeSpeed = new ChassisSpeeds();

        SwerveModulePosition[] modulesPositions = new SwerveModulePosition[] {};
        SwerveModuleState[] moduleStates = new SwerveModuleState[] {};

        ModuleInput[] moduleInputs = new ModuleInput[4];
    }

    public default void updateInputs(DriveIOInputs inputs) {
    }
    
    public default void updateSim(){
    }

    public default void resetPose(Pose2d pose){
    }
    
    public default void drive(ChassisSpeeds speeds){
    }

    public static record ModuleInput(
        boolean driveConnected,
        Angle driveRotorPosition,
        AngularVelocity driveRotorVelocity,
        Voltage driveVoltage,
        Current driveSupplyCurrent,
        Current driveStatorCurrent,
        Temperature driveTemp,

        boolean turnConnected,
        Angle turnEncoderPosition,
        AngularVelocity turnEncoderVelocity,
        Voltage turnVoltage,
        Current turnSupplyCurrent,
        Current turnStatorCurrent,
        Temperature turnTemp
    ) {}
}
