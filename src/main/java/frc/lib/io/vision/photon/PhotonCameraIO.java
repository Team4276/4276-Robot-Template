package frc.lib.io.vision.photon;

import edu.wpi.first.apriltag.AprilTag;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.vision.CameraPipeline;
import frc.robot.game.FieldLayout;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraIO extends CameraIO {

    private final PhotonCamera wrappedCamera;
    protected List<PhotonPipelineResult> lastInputBuffer = new ArrayList<>();
    private PhotonPipelineResult m_lastResult = new PhotonPipelineResult();

    public PhotonCameraIO(CameraIOConfig config) {
        super(config);
        wrappedCamera = new PhotonCamera(config.name);
    }

    public void updateToLatestResult() {
        List<PhotonPipelineResult> results = wrappedCamera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            if (m_lastResult.metadata.captureTimestampMicros <= result.metadata.captureTimestampMicros) {
                m_lastResult = result;
            }
        }

        if (results.isEmpty()) {
            lastInputBuffer = List.of();
        }
        lastInputBuffer = List.of(m_lastResult);
    }

    @Override
    public void update() {
        inputs.connected = wrappedCamera.isConnected();
        inputs.pipelineIndex = wrappedCamera.getPipelineIndex();

        lastInputBuffer = wrappedCamera.getAllUnreadResults();

        if (!lastInputBuffer.isEmpty()) {
            PhotonPipelineResult result = lastInputBuffer.get(lastInputBuffer.size() - 1);
            inputs.latestTimestamp = result.getTimestampSeconds();
            inputs.targetCount = result.targets.size();

            int[] ids = new int[result.targets.size()];
            double[] areas = new double[result.targets.size()];
            double[] pitch = new double[result.targets.size()];
            double[] yaw = new double[result.targets.size()];
            for (int i = 0; i < result.targets.size(); i++) {
                PhotonTrackedTarget target = result.targets.get(i);
                ids[i] = target.fiducialId;
                areas[i] = target.area;
                pitch[i] = target.getPitch();
                yaw[i] = target.getYaw();
            }
            inputs.targetIds = ids;
            inputs.targetAreas = areas;
            inputs.targetPitch = pitch;
            inputs.targetYaw = yaw;
            // TODO use replay inputs correctly (currently returns zeroed cam to tag)

            if (result.getTimestampSeconds() > m_lastResult.getTimestampSeconds()) {
                m_lastResult = result;
            }
        } else {
            inputs.targetCount = 0;
            inputs.targetIds = new int[0];
            inputs.targetAreas = new double[0];
            inputs.targetPitch = new double[0];
            inputs.targetYaw = new double[0];
        }

        if (Logger.hasReplaySource()) {
            lastInputBuffer = getResultsFromInputs();
        }
    }

    protected List<PhotonPipelineResult> getResultsFromInputs() {
        if (inputs.targetCount == 0) {
            return List.of();
        }
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        for (int i = 0; i < inputs.targetCount; i++) {
            targets.add(new PhotonTrackedTarget(
                    inputs.targetYaw[i],
                    inputs.targetPitch[i],
                    inputs.targetAreas[i],
                    0.0,
                    inputs.targetIds[i],
                    0,
                    0.0f,
                    new edu.wpi.first.math.geometry.Transform3d(),
                    new edu.wpi.first.math.geometry.Transform3d(),
                    0.0,
                    List.of(),
                    List.of()));
        }
        long captureTimestampMicros = (long) (inputs.latestTimestamp * 1e6);
        return List.of(new PhotonPipelineResult(
                0,
                captureTimestampMicros,
                captureTimestampMicros,
                0,
                targets));
    }

    @Override
    public void updatePipeline(CameraPipeline pipeline) {
        wrappedCamera.setPipelineIndex(pipeline.index());
    }

    public List<PhotonPipelineResult> getLastInputBuffer() {
        return lastInputBuffer;
    }

    public static int[] getIDArrayFromPhotonTargets(List<PhotonTrackedTarget> targets) {
        return (targets == null ? Stream.<PhotonTrackedTarget>empty() : targets.stream())
                .mapToInt(t -> t.fiducialId)
                .toArray();
    }

    protected static AprilTag[] getTagArrayFromPhotonTargets(List<PhotonTrackedTarget> targets) {
        return FieldLayout.getAprilTagArrayFromIDs(getIDArrayFromPhotonTargets(targets));
    }
}
