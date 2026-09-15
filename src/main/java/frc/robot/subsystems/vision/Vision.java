package frc.robot.subsystems.vision;

import java.util.Optional;

import frc.lib.bases.VisionSubsystem;
import frc.lib.util.vision.VisionEstimate;

public class Vision extends VisionSubsystem {
    public static final Vision mInstance = new Vision();

    public Vision() {
        super(VisionConstants.getName(), VisionConstants.getCameras());
    }

    @Override
    public Optional<VisionEstimate> filterEstimate(VisionEstimate estimate) {
        return super.filterEstimate(estimate);
    }
}
