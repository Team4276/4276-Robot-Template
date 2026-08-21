package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.game.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Mode;

public class AllianceFlipUtil {
    static {
        SmartDashboard.putBoolean("Sim/OverrideFlip", false);
    }

    private static boolean overrideFlip = true;

    public static Distance flipX(Distance x) {
        return FieldLayout.kFieldLength.minus(x);
    }

    public static Distance flipY(Distance y) {
        return FieldLayout.kFieldWidth.minus(y);
    }

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(flipX(translation.getMeasureX()), flipY(translation.getMeasureY()));
    }

    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.rotateBy(Rotation2d.kPi);
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    public static Distance applyX(Distance x) {
        return shouldFlip() ? flipX(x) : x;
    }

    public static Distance applyY(Distance y) {
        return shouldFlip() ? flipY(y) : y;
    }

    public static Translation2d apply(Translation2d translation) {
        return shouldFlip()
                ? flip(translation)
                : translation;
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? flip(rotation) : rotation;
    }

    public static Pose2d apply(Pose2d pose) {
        return shouldFlip()
                ? flip(pose)
                : pose;
    }

    /**
     * For SIM true sets to blue alliance
     *
     * @param shouldOverrideFlip
     */
    public static void overrideFlip(boolean shouldOverrideFlip) {
        overrideFlip = shouldOverrideFlip;
    }

    public static boolean shouldFlip() {
        overrideFlip = SmartDashboard.getBoolean("Sim/OverrideFlip", overrideFlip);

        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                && (RobotConstants.mode == Mode.SIM ? !overrideFlip : true);
    }
}
