package frc.lib.io.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.VisionEstimate;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

public abstract class VisionIO {

	protected final CameraIOConfig config;
	public final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
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
		public Pose2d[] estimatePose = new Pose2d[0];
		public double[] estimateTimestamp = new double[0];
		public int[][] estimateTagIds = new int[0][0];
	}

	public static class CameraIOConfig {
		public String name = null;
		public Pose3d robotToCameraOffset = null;
		public Vector<N3> aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);
	}

	public abstract void updateInputs();

	protected VisionIO(CameraIOConfig config) {
		this.config = config;
	}

	public void updatePipeline(CameraPipeline pipeline) {
	};

	public Optional<List<VisionEstimate>> getLastEstimates() {
		return Optional.empty();
	}

	public Vector<N3> getAprilTagStdDevs() {
		return config.aprilTagVisionStdDevs;
	}

	public void setStdDeviations(Vector<N3> standardDeviations) {
		config.aprilTagVisionStdDevs = standardDeviations;
	}

	public String getName() {
		return config.name;
	}

	public void setPipeline(CameraPipeline pipeline) {
		this.pipeline = pipeline;
		updatePipeline(pipeline);
	}
}
