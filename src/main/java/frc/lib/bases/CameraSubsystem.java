package frc.lib.bases;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.LoggedTracer;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionGamePiece;
import frc.robot.game.FieldLayout;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

public abstract class CameraSubsystem extends SubsystemBase {

    private CameraConfig config = new CameraConfig();

    private Vector<N3> m_deviations;

    private Time lastUpdatePoseTime = Seconds.of(0d), lastUpdateDetectionTime = Seconds.of(0d);
    private Pose2d lastPose = new Pose2d();
    private int numPoseStableUpdates = 0;
    private ArrayList<VisionGamePiece> tracker = new ArrayList<>();

    private Optional<VisionEstimate> lastEstimate = Optional.empty();

    private boolean enabled = true;

    public CameraSubsystem(CameraConfig config) {
        super(config.name);
        this.config = config;
        m_deviations = config.cameras[0].getAprilTagStdDevs();
    }

    private void updateDetection() {
        ArrayList<VisionGamePiece> all = getAllDetections();
        if (all.size() == 0)
            return;
        Time now = Seconds.of(Timer.getFPGATimestamp());
        tracker.removeIf((piece) -> now.minus(piece.getTimeStamp()).gte(Seconds.of(0.2)));
        while (tracker.size() > 20) {
            tracker.remove(0);
        }
        for (VisionGamePiece detection : all) {
            // if (detection.type == 0) continue;
            tracker.add(detection);
        }
    }

    private ArrayList<VisionGamePiece> getAllDetections() {
        ArrayList<VisionGamePiece> all = new ArrayList<>();
        return all;
    }

    public void applyVisionEstimate(CameraIO camera, VisionEstimate estimate) {
        // Drive.mInstance.addVisionUpdate(
        // estimate.getPose(),
        // estimate.getTimestamp(),
        // camera.getAprilTagStdDevs().times(estimate.getAverageDistance().in(Meters)));
        Drive.mInstance.addVisionMeasurement(estimate);

        if (estimate.getPose() != lastPose) {
            if (Drive.mInstance
                    .getPose()
                    .getTranslation()
                    .getDistance(estimate.getPose().getTranslation()) < config.agreedTranslationUpdateEpsilon
                            .in(Units.Meters)) {
                numPoseStableUpdates++;
            } else {
                numPoseStableUpdates = 0;
            }
        }
        lastEstimate = Optional.of(estimate);
        lastPose = estimate.getPose();
        lastUpdatePoseTime = Seconds.of(Timer.getFPGATimestamp());
    }

    private void updateLocalization() {
        for (CameraIO camera : config.cameras) {
            Optional<List<VisionEstimate>> estimatesOptional = camera.getLastEstimates();
            estimatesOptional.ifPresent(estimates -> {
                for (VisionEstimate estimate : estimates) {
                    applyVisionEstimate(camera, estimate);
                    lastUpdatePoseTime = Seconds.of(Timer.getFPGATimestamp());
                }
            });
        }
    }

    @Deprecated
    private void updatePose() {
        updateLocalization();
    }

    @Override
    public void periodic() {
        if (enabled) {
            for (CameraIO camera : config.cameras) {
                camera.update();
            }
            outputTelemetry();
            updateDetection();
            updatePose();
            Logger.recordOutput(config.name + "/Enabled", enabled);
            Logger.recordOutput(config.name + "/NumPoseStableUpdates", numPoseStableUpdates);
            Logger.recordOutput(config.name + "/LastPose", lastPose);
        }
        LoggedTracer.record(config.name);
    }

    public void setPipeline(Function<CameraIO, CameraPipeline> function) {
        for (CameraIO camera : config.cameras) {
            CameraPipeline pipelineToApply = function.apply(camera);
            camera.setPipeline(pipelineToApply);
        }
    }

    public void setPipeline(CameraPipeline pipeline) {
        setPipeline(io -> pipeline);
    }

    public void outputTelemetry() {
    }

    public Time getLastUpdatedPoseTime() {
        updatePose();
        return lastUpdatePoseTime;
    }

    public boolean getPoseStable() {
        return numPoseStableUpdates >= config.agreedTranslationUpdatesThreshold;
    }

    public boolean getPoseStable(long count) {
        return numPoseStableUpdates >= count;
    }

    public Pose2d getLatestUpdate() {
        updatePose();
        return lastPose;
    }

    public long getNumPoseStableUpdates() {
        return numPoseStableUpdates;
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

    public void disable() {
        enabled = false;
    }

    public void enable() {
        enabled = true;
    }

    public boolean getEnabled() {
        return enabled;
    }

    public void setSTDDeviations(Vector<N3> deviations) {
        for (CameraIO camera : config.cameras) {
            camera.setStdDeviations(deviations);
        }
        m_deviations = deviations;
    }

    public Command setStdDevCommand(Vector<N3> deviations) {
        return Commands.runOnce(() -> setSTDDeviations(deviations));
    }

    public Integer[] getTargetIDs() {
        Integer[] ids = {};
        if (lastEstimate.isPresent()) {
            ids = FieldLayout.getIDArrayFromAprilTagArray(lastEstimate.get().getTags());
        }
        return ids;
    }
}
