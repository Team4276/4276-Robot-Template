package frc.robot.subsystems.drive.scuffed;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIO.ModuleInput;

public class SwerveModule {
    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public final MotorIOTalonFX mDriveFx;
    public final MotorIOSparkMax mTurnSpark;

    public SwerveModule(MotorIOTalonFXConfig driveConfig, MotorIOSparkMaxConfig turnConfig) {
        mDriveFx = new MotorIOTalonFX(driveConfig);
        mTurnSpark = new MotorIOSparkMax(turnConfig);
    }

    public void setVelocity(SwerveModuleVelocity velocity) {
        mDriveFx.applySetpoint(Setpoint.withVelocitySetpoint(AngularVelocity
                .ofBaseUnits(velocity.velocity / (2 * Math.PI * DriveConstants.wheelRadiusMeters), RotationsPerSecond)));
        mTurnSpark.applySetpoint(Setpoint.withPositionSetpoint(velocity.angle.getMeasure()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public SwerveModuleVelocity getVelocity() {
        return new SwerveModuleVelocity();
    }
}
