package frc.lib.bases;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.vision.VisionEstimate;

public abstract class CameraSubsystem extends SubsystemBase {

    private CameraConfig config = new CameraConfig();

    private Vector<N3> m_deviations;

    public CameraSubsystem(CameraConfig config) {
        super(config.name);
        this.config = config;
        m_deviations = config.cameras[0].getAprilTagStdDevs();
    }

    public Optional<VisionEstimate> filterEstimate(VisionEstimate estimate) {
        return Optional.of(estimate);
    }

    public static class CameraIOConfig {
        public String name = null;
        public Pose3d robotToCameraOffset = null;
        public Vector<N3> aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);
    }

    public static class CameraConfig {
        public String name = "Vision";
        public CameraIO cameras[];
        public int agreedTranslationUpdatesThreshold;
        public Distance agreedTranslationUpdateEpsilon;
    }
}
