package frc.lib.bases;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.vision.VisionIO;
import frc.lib.util.LoggedTracer;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionGamePiece;
import frc.robot.game.FieldLayout;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

public abstract class VisionSubsystem extends SubsystemBase {

    private VisionIO[] cameras;
    private String name;

    private Vector<N3> m_deviations;

    private Time lastUpdatePoseTime = Seconds.of(0d), lastUpdateDetectionTime = Seconds.of(0d);
    private Pose2d lastPose = new Pose2d();
    private int numPoseStableUpdates = 0;
    private ArrayList<VisionGamePiece> tracker = new ArrayList<>();

    private Optional<VisionEstimate> lastEstimate = Optional.empty();

    private boolean enabled = true;

    private int agreedTranslationUpdatesThreshold;
    private Distance agreedTranslationUpdateEpsilon;

    public VisionSubsystem(String name, VisionIO[] cameras) {
        super(name);
        this.name = name;
        this.cameras = cameras;
        m_deviations = cameras[0].getAprilTagStdDevs();
        this.agreedTranslationUpdatesThreshold = 0;
        this.agreedTranslationUpdateEpsilon = Meters.of(0.1);
    }

    public VisionSubsystem(String name, VisionIO[] cameras, int agreedTranslationUpdatesThreshold,
            Distance agreedTranslationUpdateEpsilon) {
        super(name);
        this.name = name;
        this.cameras = cameras;
        m_deviations = cameras[0].getAprilTagStdDevs();
        this.agreedTranslationUpdatesThreshold = agreedTranslationUpdatesThreshold;
        this.agreedTranslationUpdateEpsilon = agreedTranslationUpdateEpsilon;
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
            tracker.add(detection);
        }
    }

    private ArrayList<VisionGamePiece> getAllDetections() {
        ArrayList<VisionGamePiece> all = new ArrayList<>();
        return all;
    }

    public void applyVisionEstimate(VisionIO camera, VisionEstimate estimate) {
        Drive.mInstance.addVisionMeasurement(estimate);

        if (estimate.getPose() != lastPose) {
            if (Drive.mInstance
                    .getPose()
                    .getTranslation()
                    .getDistance(estimate.getPose().getTranslation()) < agreedTranslationUpdateEpsilon
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
        for (VisionIO camera : cameras) {
            Optional<List<VisionEstimate>> estimatesOptional = camera.getLastEstimates();
            estimatesOptional.ifPresent(estimates -> {
                for (VisionEstimate estimate : estimates) {
                    Optional<VisionEstimate> filtered = filterEstimate(estimate);
                    filtered.ifPresent(est -> applyVisionEstimate(camera, est));
                    lastUpdatePoseTime = Seconds.of(Timer.getFPGATimestamp());
                }
            });
        }
    }

    @Override
    public void periodic() {
        if (enabled) {
            for (VisionIO camera : cameras) {
                camera.updateInputs();
                Logger.processInputs(camera.getName(), camera.inputs);
            }
            LoggedTracer.record(name);
            updateDetection();
            updateLocalization();
            Logger.recordOutput(name + "/Enabled", enabled);
            Logger.recordOutput(name + "/NumPoseStableUpdates", numPoseStableUpdates);
            Logger.recordOutput(name + "/LastPose", lastPose);
        }
    }

    public void setPipeline(Function<VisionIO, CameraPipeline> function) {
        for (VisionIO camera : cameras) {
            CameraPipeline pipelineToApply = function.apply(camera);
            camera.setPipeline(pipelineToApply);
        }
    }

    public void setPipeline(CameraPipeline pipeline) {
        setPipeline(io -> pipeline);
    }

    public Time getLastUpdatedPoseTime() {
        updateLocalization();
        return lastUpdatePoseTime;
    }

    public boolean getPoseStable() {
        return numPoseStableUpdates >= agreedTranslationUpdatesThreshold;
    }

    public boolean getPoseStable(long count) {
        return numPoseStableUpdates >= count;
    }

    public Pose2d getLatestUpdate() {
        updateLocalization();
        return lastPose;
    }

    public long getNumPoseStableUpdates() {
        return numPoseStableUpdates;
    }

    public Optional<VisionEstimate> filterEstimate(VisionEstimate estimate) {
        return Optional.of(estimate);
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

    public Command enableCommand() {
        return Commands.runOnce(() -> enable());
    }

    public Command disableCommand() {
        return Commands.runOnce(() -> disable());
    }

    public void setSTDDeviations(Vector<N3> deviations) {
        for (VisionIO camera : cameras) {
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

    public VisionIO[] getCameras() {
        return cameras;
    }
}
