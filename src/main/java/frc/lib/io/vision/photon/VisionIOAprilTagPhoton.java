package frc.lib.io.vision.photon;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.system.Timer;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.game.FieldLayout;
// import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOAprilTagPhoton extends VisionIOPhotonCamera {

	private final PhotonPoseEstimator estimator;
	private final AprilTagFieldLayout layout;

	private final PoseStrategy strategy;

	private Optional<List<VisionEstimate>> lastGivenEstimate = Optional.empty();

	public VisionIOAprilTagPhoton(CameraIOConfig config, PoseStrategy strategy, AprilTagFieldLayout layout) {
		super(config);
		this.layout = layout;
		this.strategy = strategy;
		estimator = new PhotonPoseEstimator(layout,
				new Transform3d(config.robotToCameraOffset.getTranslation(), config.robotToCameraOffset.getRotation()));
	}

	public Optional<EstimatedRobotPose> getPoseEstimateWithStrategy(PhotonPipelineResult result) {
		Optional<EstimatedRobotPose> estimate = Optional.empty();
		switch (strategy) {
			case MULTI_TAG_PNP_ON_COPROCESSOR:
				estimate = estimator.estimateAverageBestTargetsPose(result);
				return estimate;
			case PNP_DISTANCE_TRIG_SOLVE:
				estimate = estimator.estimatePnpDistanceTrigSolvePose(result);
				return estimate;
			case AVERAGE_BEST_TARGETS:
				estimate = estimator.estimateAverageBestTargetsPose(result);
				return estimate;
			case CLOSEST_TO_CAMERA_HEIGHT:
				estimate = estimator.estimateClosestToCameraHeightPose(result);
				return estimate;
			case LOWEST_AMBIGUITY:
				estimate = estimator.estimateLowestAmbiguityPose(result);
				return estimate;
			default:
				estimate = estimator.estimateLowestAmbiguityPose(result);
				return estimate;
		}
	}

	public Optional<VisionEstimate> convertRawEstimate(PhotonPipelineResult result) {
		Optional<EstimatedRobotPose> updated = getPoseEstimateWithStrategy(result);
		if (updated.isPresent()) {
			EstimatedRobotPose estimate = updated.get();
			int buffer[] = new int[estimate.targetsUsed.size()];
			for (int i = 0; i < buffer.length; i++) {
				buffer[i] = estimate.targetsUsed.get(i).fiducialId;
			}
			return Optional.of(new VisionEstimate(
					estimate.estimatedPose.toPose2d(), Seconds.of(estimate.timestampSeconds), buffer));
		}
		return Optional.empty();
	}

	public Optional<List<VisionEstimate>> getLastEstimates() {
		ArrayList<VisionEstimate> buffer = new ArrayList<>();
		if (getLastInputBuffer().isEmpty()) {
			return Optional.empty();
		}

		for (PhotonPipelineResult result : getLastInputBuffer()) {
			Optional<VisionEstimate> output = convertRawEstimate(result);
			output.ifPresent(converted -> buffer.add(converted));
		}
		return (lastGivenEstimate = buffer.isEmpty() ? Optional.empty() : Optional.of(buffer));
	}

	@Override
	public void updateInputs() {
		super.updateInputs();

		ArrayList<VisionEstimate> buffer = new ArrayList<>();
		if (!getLastInputBuffer().isEmpty()) {
			for (PhotonPipelineResult result : getLastInputBuffer()) {
				Optional<VisionEstimate> output = convertRawEstimate(result);
				output.ifPresent(converted -> buffer.add(converted));
			}
		}
		lastGivenEstimate = buffer.isEmpty() ? Optional.empty() : Optional.of(buffer);

		lastGivenEstimate.ifPresent(estimates -> {
			for (int i = 0; i < estimates.size(); i++) {
				VisionEstimate est = estimates.get(i);
				Logger.recordOutput(config.name + "/estimate/" + i + "/Pose", est.getPose());
				Logger.recordOutput(config.name + "/estimate/" + i + "/Timestamp",
						est.getTimestamp().in(Seconds));
			}
			if (!estimates.isEmpty()) {
				inputs.hasEstimate = true;
				int size = estimates.size();
				inputs.estimatePose = new Pose2d[size];
				inputs.estimateTimestamp = new double[size];
				inputs.estimateTagIds = new int[size][0];

				for (int j = 0; j < size; j++) {
					var estimate = estimates.get(j);
					inputs.estimatePose[j] = estimate.getPose();
					inputs.estimateTimestamp[j] = estimate.getTimestamp().in(Seconds);
					Integer[] tagIds = FieldLayout.getIDArrayFromAprilTagArray(estimate.getTags());
					inputs.estimateTagIds[j] = new int[tagIds.length];
					for (int k = 0; k < tagIds.length; k++) {
						inputs.estimateTagIds[j][k] = tagIds[k];
					}
				}
			}
		});

		// estimator.addHeadingData(
		// Timer.getFPGATimestamp(), Drive.mInstance.getPose().getRotation());
	}
}
