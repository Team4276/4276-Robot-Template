package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import frc.lib.io.vision.VisionIO.CameraIOConfig;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.photon.VisionIOAprilTagPhoton;
import frc.lib.io.vision.sim.VisionIOSimulated;
import frc.robot.RobotConstants;
import frc.robot.game.FieldLayout;

public class VisionConstants {

    public static final String NAME = "Vision";

    public static final Vector<N3> DEFAULT_STD_DEVIATION = VecBuilder.fill(0.3, 0.3, 99999999999.999999);

    public static final Vector<N3> ALIGN_STD_DEVATION = VecBuilder.fill(0.1, 0.1, 99999999999.999999);

    public static final PoseStrategy DEFAULT_APRIL_TAG_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final AprilTagFieldLayout LAYOUT = FieldLayout.kApriltagLayout;

    public static final class BackConstants {

        public static final PoseStrategy APRIL_TAG_STRATEGY = DEFAULT_APRIL_TAG_STRATEGY;

        public static final Pose3d OFFSET_FROM_CENTER = Pose3d.kZero;

        public static CameraIOConfig getConfig() {
            CameraIOConfig config = new CameraIOConfig();
            config.aprilTagVisionStdDevs = DEFAULT_STD_DEVIATION;
            config.robotToCameraOffset = OFFSET_FROM_CENTER;
            config.name = "photonvision";
            return config;
        }

        public static final VisionIO getIO() {
            return switch (RobotConstants.mode) {
                case REAL -> new VisionIOAprilTagPhoton(getConfig(), APRIL_TAG_STRATEGY, LAYOUT);
                case SIM -> new VisionIOSimulated(getConfig());
                case REPLAY -> new VisionIO(getConfig()) {
                    public void updateInputs() {
                    };
                };
            };
        }
    }

    public static String getName() {
        return NAME;
    }

    public static VisionIO[] getCameras() {
        return new VisionIO[] { BackConstants.getIO() };
    }
}
