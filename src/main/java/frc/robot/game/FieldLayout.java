package frc.robot.game;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;

public class FieldLayout {
    public static final AprilTagFieldLayout kApriltagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance kFieldLength = Meters.of(kApriltagLayout.getFieldLength());
    public static final Distance kFieldWidth = Meters.of(kApriltagLayout.getFieldWidth());

    public static class LinesVertical {
        public static final Distance kCenter = kFieldLength.div(2.0);
    }

    public static class LinesHorizontal {
        public static final Distance kCenter = kFieldWidth.div(2.0);
    }
}
