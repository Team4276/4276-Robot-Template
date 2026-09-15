package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import frc.lib.util.vision.VisionEstimate;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        Pose2d pose = Pose2d.kZero;
        Angle gyroAngle = Units.Degrees.of(0);
        ChassisVelocities fieldRelativeSpeed = new ChassisVelocities();
        ChassisVelocities robotRelativeSpeed = new ChassisVelocities();

        SwerveModulePosition[] modulesPositions = new SwerveModulePosition[] {};
        SwerveModuleVelocity[] moduleStates = new SwerveModuleVelocity[] {};

        ModuleInput[] moduleInputs = new ModuleInput[4];
    }

    public default void updateInputs(DriveIOInputs inputs) {
    }

    public default void updateSim() {
    }

    public default void resetPose(Pose2d pose) {
    }

    public default void drive(ChassisVelocities speeds) {
    }

    public default void addVisionMeasurement(VisionEstimate estimate) {
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
            Temperature turnTemp) {
    }
}
