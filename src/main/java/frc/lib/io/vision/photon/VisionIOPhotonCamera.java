package frc.lib.io.vision.photon;

import frc.lib.io.vision.VisionIO;
import frc.lib.util.vision.CameraPipeline;
import frc.robot.game.FieldLayout;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonCamera extends VisionIO {

    private final PhotonCamera wrappedCamera;
    protected List<PhotonPipelineResult> lastInputBuffer = new ArrayList<>();
    private PhotonPipelineResult m_lastResult = new PhotonPipelineResult();

    public VisionIOPhotonCamera(CameraIOConfig config) {
        super(config);
        wrappedCamera = new PhotonCamera(config.name);
    }

    @Override
    public void updateInputs() {
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
    }

    @Override
    public void updatePipeline(CameraPipeline pipeline) {
        wrappedCamera.setPipelineIndex(pipeline.index());
    }

    public List<PhotonPipelineResult> getLastInputBuffer() {
        return lastInputBuffer;
    }

    public static int[] getIDArrayFromPhotonTargets(List<PhotonTrackedTarget> targets) {
        return (targets == null ? java.util.stream.Stream.<PhotonTrackedTarget>empty() : targets.stream())
                .mapToInt(t -> t.fiducialId)
                .toArray();
    }

    protected static org.wpilib.vision.apriltag.AprilTag[] getTagArrayFromPhotonTargets(
            List<PhotonTrackedTarget> targets) {
        return FieldLayout.getAprilTagArrayFromIDs(getIDArrayFromPhotonTargets(targets));
    }
}
