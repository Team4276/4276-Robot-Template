package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.CameraConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.io.vision.photon.PhotonCameraIO;
import frc.lib.io.vision.sim.EmptySimulatedCameraIO;
import frc.robot.RobotConstants;
import frc.robot.game.FieldLayout;

public class VisionConstants {

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

        public static final CameraIO getIO() {
            return switch (RobotConstants.mode) {
                case REAL -> new PhotonCameraIO(getConfig());
                // case SIM -> RobotConstants.simulateVision ? new
                // AprilTagSimulatedPhotonCameraIO(
                // getVisionSystemSim(),
                // getConfig(),
                // getSimConfig(),
                // APRIL_TAG_STRATEGY)
                // : new EmptySimulatedCameraIO(getConfig());
                case REPLAY -> new EmptySimulatedCameraIO(getConfig());
                default -> new EmptySimulatedCameraIO(getConfig());
            };
        }

        // public static SimulatedCameraIOConfig getSimConfig() {
        // SimulatedCameraIOConfig config = new SimulatedCameraIOConfig();
        // config.calibErrorPx = 0.35;
        // config.calibErrorPy = 0.25;
        // config.resolutionHeightPixels = 600; // pixels
        // config.resolutionWidthPixels = 800; // pixels
        // config.kErrThreshold = Units.Degrees.of(25.0);
        // config.maxUpdateTagDistance = Units.Meters.of(1.8);
        // config.kFieldOfView = Units.Degrees.of(82.0);
        // config.side = Side.FRONT;

        // return config;
        // }
    }

    public static CameraConfig getConfig() {
        CameraConfig config = new CameraConfig();

        return config;
    }

    // TODO vision sim

    // public static VisionSystemSim getVisionSystemSim() {
    // VisionSystemSim visionSim = new VisionSystemSim("main");
    // visionSim.addAprilTags(FieldLayout.kAprilTagMap);
    // return visionSim;
    // }
}
