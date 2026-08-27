package frc.robot.subsystems.vision;

import java.util.Optional;

import frc.lib.bases.CameraSubsystem;
import frc.lib.util.vision.VisionEstimate;

public class Vision extends CameraSubsystem {
    public static final Vision mInstance = new Vision();

    public Vision() {
        super(VisionConstants.getConfig());
    }

    @Override
    public Optional<VisionEstimate> filterEstimate(VisionEstimate estimate) {
        return super.filterEstimate(estimate);
    }
}
