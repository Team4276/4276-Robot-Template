package frc.lib.io.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionGamePiece;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class CameraIO {

	protected final CameraIOConfig config;
	protected final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
	protected CameraPipeline pipeline = CameraPipeline.getDefault();

	@AutoLog
	public static class VisionIOInputs {
		public boolean connected = false;
		public int pipelineIndex = 0;
		public double latestTimestamp = 0.0;

		public int targetCount = 0;
		public int[] targetIds = new int[0];
		public double[] targetAreas = new double[0];
		public double[] targetPitch = new double[0];
		public double[] targetYaw = new double[0];

		public boolean hasEstimate = false;
		public Pose2d estimatePose = new Pose2d();
		public double estimateTimestamp = 0.0;
		public int[] estimateTagIds = new int[0];
	}

	protected CameraIO(CameraIOConfig config) {
		this.config = config;
	}

	public abstract void updatePipeline(CameraPipeline pipeline);

	public Optional<List<VisionGamePiece>> getAllGamePieces() {
		return Optional.empty();
	}

	@Deprecated
	public Optional<VisionEstimate> getRobotPose() {
		return Optional.empty();
	}

	public Optional<List<VisionEstimate>> getLastEstimates() {
		return Optional.empty();
	}

	public Vector<N3> getAprilTagStdDevs() {
		return config.aprilTagVisionStdDevs;
	}

	public void setStdDeviations(Vector<N3> standardDeviations) {
		config.aprilTagVisionStdDevs = standardDeviations;
	}

	public void setPipeline(CameraPipeline pipeline) {
		this.pipeline = pipeline;
		updatePipeline(pipeline);
	}

	public void update() {
		Logger.processInputs(config.name, inputs);
	}
}
